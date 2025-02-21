#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include "examples_common.h" 


int main(int argc, char** argv) {
  // Check if there are exactly 19 arguments.
  // 1) robot hostname
  // 2) global-speed-factor
  // 3) t_start, 4) t_end
  // 5-11) q_start_1 to q_start_7
  // 12-18) q_end_1 to q_end_7
  if (argc != 19) {
    std::cerr << "Usage: " << argv[0]
              << " <robot-hostname> <global-speed-factor> <t_start> <t_end> "
                 " <q_start_1> ... <q_start_7> <q_end_1> ... <q_end_7>" 
              << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    double global_speed_factor = std::stod(argv[2]);
    
    double t_start = std::stod(argv[3]);
    double t_end = std::stod(argv[4]);
    
    // get q start and end position
    std::array<double,7> q_start_array;
    std::array<double,7> q_end_array;
    for (int i = 0; i < 7; i++) {
      q_start_array[i] = std::stod(argv[5 + i]);
      q_end_array[i] = std::stod(argv[12 + i]);
    }

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    #include "linear interpolation.txt" 
    
     // Retrieve the current robot state once, to get the current joint
    franka::RobotState current_state = robot.readOnce();
    std::array<double, 7> q_current = current_state.q;

    std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
      std::cin.ignore();

    // Iterate through each point in the trajectory.
    for (size_t i = 0; i < trajectory.size(); i++) {
      const auto& point = trajectory[i];
      double T;
      if (i == 0){
        T = point.time;
      } else {
        const auto& prev_point = trajectory[i - 1];
        T = point.time - prev_point.time;
      }
      
      std::array<double, 7> q_goal = point.q;

      double max_speed = 2.175;
      double max_diff = 0.0;
      // Determine the largest joint difference between the current and goal positions.
      for (size_t j = 0; j < 7; j++) {
        double diff = std::fabs(q_goal[j] - q_current[j]);
        if (diff > max_diff) {
          max_diff = diff;
        }
      }

      double speed_factor = (T > 0.0) ? (max_diff / T) / max_speed : global_speed_factor;
      // Limit the speed factor by the global_speed_factor.
      if (speed_factor > global_speed_factor) {
        speed_factor = global_speed_factor; 
      }
      if (speed_factor <= 0.0) {
        speed_factor = 0.1 * global_speed_factor; 
      }
 
      // Print status info every 200 points (or at i=0).
      if (i % 20 == 0) {
        std::cout << "Moving to: ";
        for (double qv : q_goal) {
          std::cout << qv << " ";
        }
        std::cout << " at t=" << point.time << "s with speed_factor=" << speed_factor << std::endl;
      }

      MotionGenerator motion_generator(speed_factor, q_goal);

      robot.control(motion_generator);

      q_current = q_goal;
    }

    std::cout << "Finished moving through the trajectory!" << std::endl;

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
