#include <iostream>
#include <array>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstdlib>

#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

// Defines astruct to store each sampled point of the trajectory
struct TrajectoryPoint {
  double time;                   
  std::array<double, 7> q;       
  std::array<double, 7> dq;      
  std::array<double, 7> ddq;    
};

#include "linear_trajectory.txt"

int main(int argc, char** argv){
  // Check if there are exactly 17 arguments.
  // 1) robot hostname
  // 2) global-speed-factor
  // 3) Total time
  // 4-10) q_start_1 to q_start_7
  // 11-17) q_end_1 to q_end_7
  
  if (argc!= 18){
    std::cerr << " Usage: " << argv[0]
              << " <robot-hostname> <t_start> <t_end>"
                 " <q_start_1> ... <q_start_7> <q_end_1> ... <q_end_7>" 
              << std::endl;
    return -1;
  }

  try{
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    double t_start = std::stod(argv[2]);
    double t_end   = std::stod(argv[3]);

    // get q_start and q_end positions
    std::array<double, 7> q_start{}, q_end{};
    for (int i = 0; i < 7; i++){
      q_start[i] = std::stod(argv[4 + i]);
      q_end[i] =std::stod(argv[11 + i]);
    }

    if (t_start <= 0.0 || t_end <= t_start) {
      std::cerr << "Error: t_start must be > 0 and t_end > t_start" << std::endl;
      return -1;
    }
    double dt = 0.001;  // 1ms
    std::vector<TrajectoryPoint> trajectory =
    generateLinearTrajectory(t_start, t_end, q_start, q_end);

    if (trajectory.empty()) {
    std::cerr << "Error: No valid trajectory points provided!" << std::endl;
    return -1;
    }

    // First move the robot to a suitable joint comfiguration
    MotionGenerator motion_generator(0.3, q_start);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to start joint configuration." << std::endl;
    
    franka::Model model = robot.loadModel();

    size_t traj_size = trajectory.size();

    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    
    // Define the PD gains Kp and Kd
    Eigen::Matrix<double, 7, 1> Kp;
    Eigen::Matrix<double, 7, 1> Kd;
    Kp<< 100.0, 100.0, 100.0, 180.0, 10.0, 100.0, 10.0;
    Kd<< 20.0, 20.0, 20.0, 20.0, 25.0, 20.0, 15.0;

    std::vector<double> time_log;
    std::vector<std::array<double, 7>> q_ref_log;
    std::vector<std::array<double, 7>> q_act_log;
    
    // initialize a time variable
    double time = 0.0;

    auto torque_control_callback =[&](const franka::RobotState robot_state,
                                      franka::Duration period) -> franka::Torques {
      time += period.toSec();
      
      // Computes the current trajectory index
      size_t traj_index = static_cast <size_t> (time / dt);
      if (traj_index >= traj_size) {
        std::cout << "Trajectory finished. Stopping controller..." << std::endl;
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
      }

      // get q_ref, dq_ref, ddq_ref from trajectory
      TrajectoryPoint ref = trajectory[traj_index];

      // Current actual joint angle and joint angular velocity
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      Eigen::Matrix<double, 7, 1> q_ref, dq_ref, ddq_ref;
      for (int i = 0; i < 7; i++) {
        q_ref(i)   = ref.q[i];
        dq_ref(i)  = ref.dq[i];
        ddq_ref(i) = ref.ddq[i];
      }

      // e = q_ref - q, de = dq_ref - dq
      Eigen::Matrix<double, 7, 1> e   = q_ref  - q;
      Eigen::Matrix<double, 7, 1> de  = dq_ref - dq;
      
      // get model parameters
      std::array<double, 49> mass_array = model.mass(robot_state);
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 7> gravity_array = model. gravity(robot_state);

      // Real states of the end effector, convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> M (mass_array.data()); 
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> C (coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> G (gravity_array.data());

      Eigen::Matrix<double, 7, 1> U_ff = M * ddq_ref + C + G;
      
      Eigen::Matrix<double, 7, 1> U_fb = Kp.cwiseProduct(e) + Kd.cwiseProduct(de);

      Eigen::Matrix<double, 7, 1> U_cmd = U_ff + U_fb;

      {
        time_log.push_back(time);
        std::array<double, 7> tmp_q_ref{}, tmp_q_act{};
        for (int i = 0; i < 7; i++){
          tmp_q_ref[i] = q_ref(i);
          tmp_q_act[i] = q(i);
        }
        q_ref_log.push_back(tmp_q_ref);
        q_act_log.push_back(tmp_q_act);
      }

      // Convert the Eigen torque to a standard array
      std::array<double, 7> u_cmd_array{};
      Eigen::VectorXd::Map(&u_cmd_array[0], 7) = U_cmd;
     
      return u_cmd_array;
    };

    std::cout << "WARNING: This example will move the robot!\n"
                 "Make sure you have the user stop button at hand!\n"
                 "Press Enter to continue..."
              << std::endl;
    std::cin.ignore();

    robot.control(torque_control_callback);

    std::cout << "Finished moving through the trajectory!" << std::endl;

    {
      std::ofstream csv_file("trajectory_data.csv");
      csv_file << "time";
      for (int j = 0; j < 7; j++) {
        csv_file << ",q_ref_" << j << ",q_act_" << j;
      }
      csv_file << "\n";

      for (size_t i = 0; i < time_log.size(); i++) {
        csv_file << time_log[i];
        for (int j = 0; j < 7; j++) {
          csv_file << "," << q_ref_log[i][j] << "," << q_act_log[i][j];
        }
        csv_file << "\n";
      }
      csv_file.close();
      std::cout << "Saved trajectory data to trajectory_data.csv\n";
      
      // create a Gnuplot 
      std::ofstream gp("plot_trajectories.gp");
      
      // Set terminal type, output file path, and CSV delimiter
      gp << "set terminal pngcairo size 1200,1000\n";
      gp << "set output 'trajectories Errors.png'\n";
      gp << "set datafile separator ','\n";

      
      //  Use multiplot to divide the canvas into multiple subplots
      // Divides the canvas into a 4×2 grid (8 total subplots)
      // 'rowsfirst' means the subplots will be filled row by row.
      gp << "set multiplot layout 4,2 rowsfirst\n\n";

      // Loop 7 times, creating one subplot for each joint
      for (int j = 0; j < 7; j++) {
        
        // Calculates the column indices in the CSV for "q_ref_j" and "q_act_j" based on joint number j.
        // - Column 1 is time
        // - Column 2 is q_ref_0, Column 3 is q_act_0
        // - Column 4 is q_ref_1, Column 5 is q_act_1
        // ...
        // For joint j:
        // q_ref_j is in column (2 + 2*j), and q_act_j is in column (3 + 2*j)

      // Set subplot title and axis labels
      gp << "set title 'Joint " << j << " Error (q_r - q)'\n";

      gp << "set xlabel 'Time (s)'\n";
      gp << "set ylabel 'Error (rad)'\n";

      
  
      gp << "set grid xtics ytics\n";
      
      int col_ref = 2 + j * 2;  
      int col_act = 3 + j * 2; 

      // Plot two curves in the current subplot: reference vs. actual
      gp << "plot 'trajectory_data.csv' using 1:($" 
         << col_ref << " - $" << col_act << ") "
         << " with lines lw 2 title 'Error_j" << j << "'\n\n";
    }
      
      //  Exit the multiplot mode
      gp << "unset multiplot\n";
      gp.close();

      // Call the gnuplot command to execute the script "plot_trajectories.gp"
      int ret = system("gnuplot plot_trajectories.gp");
      if (ret == 0) {
        std::cout << "Generated 'trajectories Errors.png' using gnuplot.\n";
      } else {
        std::cerr << "Failed to call gnuplot. (ret=" << ret << ")\n";
      }
    }
    
  } catch(const std::exception& e){
    std::cerr << e.what() << std::endl;
    return -1;
  }
   
  return 0;
}