#include <array>
#include <cmath>
#include <iostream>
#include <chrono>

#include <vector>
#include <fstream>
#include <cstdio>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/duration.h>
#include "examples_common.h"  

// quintic polynomial interpolation factor
// s(t) = 10*t^3 - 15*t^4 + 6*t^5, with t in [0,1]
// s(0)=0, s'(0)=0, s''(0)=0; s(1)=1, s'(1)=0, s''(1)=0
double quintic_s(double tau) {
  return 10 * std::pow(tau,3) - 15 * std::pow(tau,4) + 6 * std::pow(tau,5);
}

std::array<double, 4> eulerXYZToQuaternion(double roll, double pitch, double yaw){
    
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    std::array<double, 4> q;
    //Quaternion[w, x, y, z]
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    return q;
}

std::array<double, 4> matrixToQuaternion(const std::array<double, 16>& pose){
    //Ratation matrix
    double r00 = pose[0];  double r01 = pose[1];  double r02 = pose[2];
    double r10 = pose[4];  double r11 = pose[5];  double r12 = pose[6];
    double r20 = pose[8];  double r21 = pose[9];  double r22 = pose[10];

    std::array<double, 4> q;

    double trace = r00 + r11 + r22;
    //trace > 0
    if (trace > 0.0){
        double s = std::sqrt(trace + 1.0) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (r21 - r12) / s;
        q[2] = (r02 - r20) / s;
        q[3] = (r10 - r01) / s;
    } else { //trace < 0
      if((r00 > r11)&(r00 > r22)){
        double s = std::sqrt(1.0 + r00 - r11 -r22) * 2.0;
        q[0] = (r21 - r12) / s;
        q[1] = 0.25 * s;
        q[2] = (r01 + r10) / s;
        q[3] = (r02 + r20) / s;
      } else if (r11 > r22) {
        double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        q[0] = (r02 - r20) / s;
        q[1] = (r01 + r10) / s;
        q[2] = 0.25 * s;
        q[3] = (r12 + r21) / s;
      } else {
        double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        q[0] = (r10 - r01) / s;
        q[1] = (r02 + r20) / s;
        q[2] = (r12 + r21) / s;
        q[3] = 0.25 * s;
      }
    }
    return q;
}

//initial Quaternion q0, goal Quaternion q1
std::array<double, 4> slerp(const std::array<double, 4>& q0, const std::array<double, 4>& q1, double t) {
   // q0, q1: [w, x, y, z]
  auto norm = [](const std::array<double, 4>& q) {
    //length from quaternion
    return std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  };

  //normalize q0 & q1
  auto normalize = [&](const std::array<double,4>& q) {
    std::array<double, 4> spherical;
    double n = norm(q);
    if (n < 1e-8) return q; // avoid division by 0
    spherical[0] = q[0]/n; spherical[1] = q[1]/n; spherical[2] = q[2]/n; spherical[3] = q[3]/n;
    return spherical;
  };

  std::array<double,4> qa = normalize(q0);
  std::array<double,4> qb = normalize(q1);

  //dot = wa * wb + xa * xb + ya * yb + za * zb 
  double dot = qa[0]*qb[0] + qa[1]*qb[1] + qa[2]*qb[2] + qa[3]*qb[3];
  //difference bigger than 90 degrees, 
  if (dot < 0.0) {
  qb[0] = -qb[0]; qb[1] = -qb[1];
  qb[2] = -qb[2]; qb[3] = -qb[3];
  dot = -dot;
  }

  const double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD) {
  //Quaternions are close
  std::array<double,4> linear;
  for (int i=0; i<4; i++) {
    linear[i] = qa[i] + t * (qb[i] - qa[i]); //lerp
  }
  return normalize(linear);
  }

  //qa • qb = ||qa|| * ||qb|| * cos(θ) -> qa • qb = cos(θ) (angle between two Quaternion)
  //θ0 = arccos(qa • qb)
  double theta_0 = std::acos(dot);   
  double theta = theta_0 * t;   
  double sin_theta_0 = std::sin(theta_0);
  double sin_theta = std::sin(theta);

  //q(t) = (sin((1 - t) * θ0) / sin(θ0)) * qa + (sin(tθ0) / sin(θ0)) * qb
  //q(t) = s0 * qa + s1 * qb
  double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  double s1 = sin_theta / sin_theta_0;

  std::array<double,4> spherical;
  for (int i=0; i<4; i++) {
    spherical[i] = (s0 * qa[i]) + (s1 * qb[i]); //slerp
  }
  return spherical;
}

void quaternionToMatrix(const std::array<double, 4>& q, std::array<double, 16>& pose){
    // q: [w, x, y, z]
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    double xx = x * x; double yy = y * y; double zz = z * z;
    double xy = x * y; double xz = x * z; double yz = y * z;
    double wx = w * x; double wy = w * y; double wz = w * z;

    pose[0] = 1.0 - 2.0 * (yy + zz); //r00
    pose[1] = 2.0 * (xy - wz); //r01
    pose[2] = 2.0 * (xz + wy); //r02
    //pose[3] = 0.0

    pose[4] = 2.0 * (xy + wz); //r10
    pose[5] = 1.0 - 2.0 * (xx + zz); //r11
    pose[6] = 2.0 * (yz - wx); //r12
    //pose[7] = 0.0

    pose[8] = 2.0 * (xz - wy); //r20
    pose[9] = 2.0 * (yz + wx); //r21
    pose[10] = 1.0 - 2.0 * (xx + yy); //r22
    //pose[11] = 0.0
}

static std::array<double, 3> quaternionToEulerXYZ(const std::array<double, 4>& q) {
  double w = q[0], x = q[1], y = q[2], z = q[3];

  // Normalize
  double n = std::sqrt(w*w + x*x + y*y + z*z);
  if (n < 1e-9) {
    return {0.0, 0.0, 0.0};
  }
  w /= n; x /= n; y /= n; z /= n;

  // roll (X)
  double sinr_cosp = 2.0 * (w*x + y*z);
  double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (Y)
  double sinp = 2.0 * (w*y - z*x);
  double pitch = 0.0;
  if (std::fabs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  // yaw (Z)
  double siny_cosp = 2.0 * (w*z + x*y);
  double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

//<robot-hostname> <x> <y> <z> <roll> <pitch> <yaw>
int main(int argc, char** argv) {
  if (argc != 8){
    std::cerr <<"Usage: " << argv[0]  
              << " <robot-hostname> <x> <y> <z> <roll> <pitch> <yaw>"
                 " All angles in radians"
              << std::endl;
    return -1;
  }

  try{
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    double x = std::stod(argv[2]);
    double y = std::stod(argv[3]);
    double z = std::stod(argv[4]);
    double roll  = std::stod(argv[5]);   
    double pitch = std::stod(argv[6]);   
    double yaw   = std::stod(argv[7]);   
  
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::RobotState initial_state = robot.readOnce();
    std::array<double, 16> initial_pose = initial_state.O_T_EE_c; 

    double initial_x = initial_pose[12];
    double initial_y = initial_pose[13];
    double initial_z = initial_pose[14];

    //rotation matrix from initial pose to Quaternion
    std::array<double, 4> quaternion_init = matrixToQuaternion(initial_pose);

    //Converts the inputs value from roll, pitch, yaw to Quaternion
    std::array<double, 4> quaternion_target = eulerXYZToQuaternion(roll, pitch, yaw);

    double dx = x - initial_x;
    double dy = y - initial_y;
    double dz = z - initial_z;

    double total_time = 10.0;

    static std::vector<std::array<double, 7>> trajectory_data; 
    trajectory_data.reserve(10000);
 
    robot.control([=](const franka::RobotState& robot_state, 
                      franka::Duration period) -> franka::CartesianPose {
      static double time = 0.0;
      time += period.toSec();

      // Normalize time parameter
      double tau = time / total_time;
      if (tau > 1.0) { tau = 1.0; }

      //s(t) = quintic_s(tau)
      double s = quintic_s(tau);
      
      // Compute the desired position using quintic interpolation
      double current_x = initial_x + s * dx;
      double current_y = initial_y + s * dy;
      double current_z = initial_z + s * dz;

      std::array<double, 4> q_current = slerp(quaternion_init, quaternion_target, s);

      std::array<double, 16> new_pose = robot_state.O_T_EE_c;
      // franka::CartesianPose new_pose(robot_state.O_T_EE_c);
      quaternionToMatrix(q_current, new_pose);
      
      new_pose[12] = current_x;
      new_pose[13] = current_y;
      new_pose[14] = current_z;
      new_pose[15] = 1.0;

      std::array<double, 3> rpy = quaternionToEulerXYZ(q_current); 
      // time, x, y, z, roll, pitch, yaw
      trajectory_data.push_back({time, current_x, current_y, current_z, 
                                 rpy[0], rpy[1], rpy[2]});

      std::cout << "Time: " << time 
                << "s, s(tau)=" << s << ", Pos=(" 
                << current_x << "," << current_y << "," << current_z << ") "
                << std::endl;
        
      if (time >= total_time) {
        std::cout << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });

    // time, x, y, z, roll, pitch, yaw
    std::string csv_file = "robot_trajectory.csv";
    {
      std::ofstream file(csv_file);
      if (!file.is_open()) {
        std::cerr << "Failed to open file " << csv_file << " for writing!" << std::endl;
        return -1;
      }
      file << "time,x,y,z,roll,pitch,yaw\n";
      for (const auto& row : trajectory_data) {
        file << row[0] << ","   // time
             << row[1] << ","   // x
             << row[2] << ","   // y
             << row[3] << ","   // z
             << row[4] << ","   // roll
             << row[5] << ","   // pitch
             << row[6] << "\n"; // yaw
      }
      file.close();
    }
    std::cout << "Trajectory data saved to " << csv_file << std::endl;

    // Call Gnuplot to generate a PNG with 6 subplots: x(t), y(t), z(t), roll(t), pitch(t), yaw(t)
    {
      FILE* gpipe = popen("gnuplot -persistent", "w");
      if (!gpipe) {
        std::cerr << "Failed to open Gnuplot pipe!" << std::endl;
      } else {
        // Set up the PNG output
        fprintf(gpipe, "set terminal pngcairo size 1200,800\n");
        fprintf(gpipe, "set output 'trajectory_plots.png'\n");
        fprintf(gpipe, "set datafile separator comma\n");

        // We'll use multiplot with layout 2 rows x 3 columns
        fprintf(gpipe, "set multiplot layout 2,3 title 'End-Effector Trajectory' font ',14'\n");

        // Subplot 1: x(t)
        fprintf(gpipe, "set title 'X(t)'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'X (m)'\n");
        fprintf(gpipe, "plot '%s' using 1:2 with lines title 'X'\n", csv_file.c_str());

        // Subplot 2: y(t)
        fprintf(gpipe, "set title 'Y(t)'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'Y (m)'\n");
        fprintf(gpipe, "plot '%s' using 1:3 with lines title 'Y'\n", csv_file.c_str());

        // Subplot 3: z(t)
        fprintf(gpipe, "set title 'Z(t)'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'Z (m)'\n");
        fprintf(gpipe, "plot '%s' using 1:4 with lines title 'Z'\n", csv_file.c_str());

        // Subplot 4: roll(t)
        fprintf(gpipe, "set title 'Roll(t)'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'Roll (rad)'\n");
        fprintf(gpipe, "plot '%s' using 1:5 with lines title 'Roll'\n", csv_file.c_str());

        // Subplot 5: pitch(t)
        fprintf(gpipe, "set title 'Pitch(t)'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'Pitch (rad)'\n");
        fprintf(gpipe, "plot '%s' using 1:6 with lines title 'Pitch'\n", csv_file.c_str());

        // Subplot 6: yaw(t)
        fprintf(gpipe, "set title 'Yaw(t)'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'Yaw (rad)'\n");
        fprintf(gpipe, "plot '%s' using 1:7 with lines title 'Yaw'\n", csv_file.c_str());

        // Close multiplot
        fprintf(gpipe, "unset multiplot\n");

        // Close the pipe so gnuplot will generate the PNG and exit
        pclose(gpipe);
        std::cout << "Generated 'Euler_angle_plots.png' with 6 subplots (x,y,z,roll,pitch,yaw).\n";
      }
    }

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
      
}

