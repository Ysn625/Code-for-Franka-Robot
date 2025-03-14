#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <chrono>
#include <fstream>
#include <cstdio>       
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

// quintic polynomial interpolation factor
// s(t) = 10*t^3 - 15*t^4 + 6*t^5, with t in [0,1]
// s(0)=0, s'(0)=0, s''(0)=0; s(1)=1, s'(1)=0, s''(1)=0
double quintic_s(double tau) {
  return 10*std::pow(tau,3) - 15*std::pow(tau,4) + 6*std::pow(tau,5);
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    constexpr double X_MIN = -0.855;
    constexpr double X_MAX = 0.855;
    constexpr double Y_MIN = -0.855;
    constexpr double Y_MAX = 0.855;
    constexpr double Z_MIN = 0.0;
    constexpr double Z_MAX = 1.0;

    double x = std::stod(argv[2]);
    double y = std::stod(argv[3]);
    double z = std::stod(argv[4]);
    
    // Move the robot to an initial joint configuration before starting Cartesian interpolation
    if (x < X_MIN || x > X_MAX ||
        y < Y_MIN || y > Y_MAX ||
        z < Z_MIN || z > Z_MAX) {
      std::cerr << "Error: Position (" << x << ", " << y << ", " << z
                << ") is out of the robot's workspace." << std::endl;
      return -1;
    }

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.3, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},       {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},       {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Define total motion time and initial state
    double total_time = 5.0;
    franka::RobotState initial_state = robot.readOnce();
    std::array<double,16> initial_pose = initial_state.O_T_EE_c; 

    double initial_x = initial_pose[12];
    double initial_y = initial_pose[13];
    double initial_z = initial_pose[14];

    double dx = x - initial_x;
    double dy = y - initial_y;
    double dz = z - initial_z;

    // Use a vector to record {time, x, y, z}
    std::vector<std::array<double,4>> trajectory_data;
    trajectory_data.reserve(5000); // Pre-allocate space for about 5s at ~1kHz

    robot.control([&](const franka::RobotState&,
                      franka::Duration period) -> franka::CartesianPose {
      static double time = 0.0;
      time += period.toSec();

      // Normalize time parameter
      double tau = time / total_time;
      if (tau > 1.0) {
        tau = 1.0;
      }
    
      // Compute the desired position using quintic interpolation
      double s = quintic_s(tau);

      double current_x = initial_x + s * dx;
      double current_y = initial_y + s * dy;
      double current_z = initial_z + s * dz;

      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] = current_x;
      new_pose[13] = current_y;
      new_pose[14] = current_z;
      
      // Record the current time and position
      trajectory_data.push_back({time, current_x, current_y, current_z});

      //Print progress information to the console
      std::cout << "Time: " << time
                << ", Position: (" << current_x << ", "
                                    << current_y << ", "
                                    << current_z << ")"
                << std::endl;
      
      // End motion if total time is reached
      if (time >= total_time) {
        std::cout << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }

      return new_pose;
    });
    
    // Write the trajectory data to a CSV file
    std::string csv_file = "robot_trajectory.csv";
    {
      std::ofstream file(csv_file);
      if (!file.is_open()) {
        std::cerr << "Failed to open file " << csv_file << " for writing!" << std::endl;
        return -1;
      }
      // Write header
      file << "time,x,y,z\n";
      for (const auto& row : trajectory_data) {
        file << row[0] << "," << row[1] << "," << row[2] << "," << row[3] << "\n";
      }
      file.close();
    }
    std::cout << "Trajectory data saved to " << csv_file << std::endl;

    // generate a PNG
    {
      FILE* gpipe = popen("gnuplot -persistent", "w"); // Opens a pipe to run "gnuplot -persistent"
      if (!gpipe) {
        std::cerr << "Failed to open Gnuplot pipe!" << std::endl;
      } else {
        // Set the output to a PNG file
        fprintf(gpipe, "set terminal pngcairo size 1000,600\n");
        fprintf(gpipe, "set output 'xyz_trajectory.png'\n");
        fprintf(gpipe, "set datafile separator comma\n");  

        // Plot all three axes (X, Y, Z) on the same graph
        fprintf(gpipe, "set title 'XYZ Trajectory'\n");
        fprintf(gpipe, "set xlabel 'Time (s)'\n");
        fprintf(gpipe, "set ylabel 'Position (m)'\n");
        fprintf(gpipe, "plot '%s' using 1:2 with lines title 'X',\\\n", csv_file.c_str());
        fprintf(gpipe, "     '%s' using 1:3 with lines title 'Y',\\\n", csv_file.c_str());
        fprintf(gpipe, "     '%s' using 1:4 with lines title 'Z'\n", csv_file.c_str());

        // Close the pipe so gnuplot will execute and create the PNG
        pclose(gpipe);
        std::cout << "Generated figure: xyz_trajectory.png" << std::endl;
      }
    }

    return 0;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
