#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"


                
    }
    try{
      franka::Robot robot(argv[1]);
      setDefaultBehavior(robot);

      std::array<double, 16> inintial_pose = robot.readOnce().O_T_EE;
      std::array<double, 16> new_pose = inintial_pose;
      
      constexpr double X_MIN = -0.855;
      constexpr double X_MAX = 0.855;
      constexpr double Y_MIN = -0.855;
      constexpr double Y_MAX = 0.855;
      constexpr double Z_MIN = 0.0;
      constexpr double Z_MAX = 1.0;

      double x = std::stod(argv[2]);
      double y = std::stod(argv[3]);
      double z = std::stod(argv[4]);
      
      if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX || z < Z_MIN || z > Z_MAX)
      {
        std::cerr << "Error: Position (" << x << ", " << y << ", " << z << ") is out of the robot's workspace." << std::endl;
        return -1;
      }

      std::cout<<"receive input variable"<<std::endl;

      new_pose[12] = x;
      new_pose[13] = y;
      new_pose[14] = z;

      std::cout << "WARNING: This example will move the robot! "
            << "Please make sure to have the user stop button at hand!" << std::endl
            << "Press Enter to continue..." << std::endl;
      std::cin.ignore();
      std::cout << "Motion finished" << std::endl;
      
      robot.setCollisionBehavior(
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

      std::cout<<"add new_pose, set collision"<<std::endl;

      robot.control([&new_pose](const franka::RobotState&, 
                               franka::Duration) -> franka::CartesianPose{
        return franka::MotionFinished(new_pose);
      });
  
    }catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}


//

         if (time == 0.0) {
            initial_pose = robot_state.O_T_EE;
         }
 
         std::array<double, 16> new_pose = initial_pose;

         double alpha = std::min(time /total_time, 1.0);

         new_pose[12] = initial_x + alpha * (x - initial_x);
         new_pose[13] = initial_y + alpha * (y - initial_y);
         new_pose[14] = initial_z + alpha * (z - initial_z);

         std::cout << "Time:" << time << ", Positon:(" << new_pose[12] << "," << new_pose[13] << "," << new_pose[14] << ")" << std::endl;

         if (time >= total_time){
          std::cout << std::endl << "Motion completed. Target reached" << std::endl;
          return franka::MotionFinished(new_pose);
         }
         return new_pose;

//
time += period.toSec();

         if (time == 0.0) {
            initial_pose = robot_state.O_T_EE;
         }

         double angle = M_PI / 4 * (1.0 - std::cos(M_PI / total_time * time));
         double delta_x = (target_x - initial_x) * (1.0 - std::cos(angle));
         double delta_y = (target_y - initial_y) * (1.0 - std::cos(angle));
         double delta_z = (target_z - initial_z) * (1.0 - std::cos(angle));

         std::array<double, 16> new_pose = initial_pose;

         new_pose[12] = initial_x + delta_x;
         new_pose[13] = initial_y + delta_y;
         new_pose[14] = initial_z + delta_z;

         std::cout << "Time:" << time
                   << ", Positon:(" << new_pose[12] << "," << new_pose[13] << "," << new_pose[14] << ")" << std::endl;

         if (time >= total_time){
          std::cout << std::endl << "Motion completed. Target reached" << std::endl;
          return franka::MotionFinished(new_pose);
         }
         return new_pose;
       

//
 time += period.toSec();

         if (time == 0.0) {
            initial_pose = robot_state.O_T_EE;
         }

         double alpha = 0.5 * (1 - std::cos(M_PI * time / total_time));

         std::array<double, 16> new_pose = initial_pose;

         new_pose[12] = initial_x + alpha * (x - initial_x);
         new_pose[13] = initial_y + alpha * (y - initial_y);
         new_pose[14] = initial_z + alpha * (z - initial_z);

         std::cout << "Time:" << time
                   << ", Positon:(" << new_pose[12] << "," << new_pose[13] << "," << new_pose[14] << ")" << std::endl;

         if (time >= total_time){
          std::cout << std::endl << "Motion completed. Target reached" << std::endl;
          return franka::MotionFinished(new_pose);
         }
         return new_pose;