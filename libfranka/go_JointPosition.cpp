/**
 * go_JointPosition.cpp
 *
 * A minimal script for moving to a specific joint position, by finding a minimum-jerk trajectory! Assumes no objects,
 * or collision-checking... simply interpolates between states.
 *
 * See following for minimum-jerk explanation: https://mika-s.github.io/python/control-theory/trajectory-generation/
 *                                                  2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
 */

#include <iostream>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

class JointPositionController {
  public:
    JointPositionController();
    JointPositionController(const std::array<double, 7> q);
    JointPositionController(const std::array<double, 7> q, const double t);
    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);
  private:
    double time = 0.0;
    double time_goal;
    double time_inter;
    double smooth_inter;
    std::array<double, 7> q_start;
    std::array<double, 7> q_goal;
    std::array<double, 7> q_inter;
    std::array<double, 7> delta_q;
};

JointPositionController::JointPositionController() {
  q_goal = {{0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4}};
  time_goal = 2.5;
}

JointPositionController::JointPositionController(const std::array<double, 7> q) {
  q_goal = q;
  time_goal = 2.5;
}

JointPositionController::JointPositionController(const std::array<double, 7> q, const double t) {
  q_goal = q;
  time_goal = t;
}

franka::JointPositions JointPositionController::operator()(const franka::RobotState& robot_state,
                                                           franka::Duration period) {
  time += period.toSec();

  if (time == 0.0) {
    q_start = robot_state.q_d;

    for (int i = 0; i < 7; i++) {
      delta_q[i] = q_goal[i] - q_start[i];
    }

    franka::JointPositions q_c(q_start);
    return q_c;

  } else if (time >= time_goal) {

    franka::JointPositions q_c(q_goal);
    return franka::MotionFinished(q_c);

  } else {
    time_inter = time / time_goal;

    smooth_inter = 10 * pow(time_inter, 3.0) - 15 * pow(time_inter, 4.0) + 6 * pow(time_inter, 5.0);

    for (int i = 0; i < 7; i++) {
      q_inter[i] = q_start[i] + delta_q[i] * smooth_inter;
    }

    franka::JointPositions q_c(q_inter);
    return q_c;
  }

}


int main(int argc, char** argv) {
  if (argc != 2 && argc != 9 && argc != 10) {
    std::cerr << "Usage: "
              << "IP of robot"
              << "<joint1> <joint2> <joint3> <joint4> <joint5> <joint6> <joint7>"
              << "<motion-time>" << std::endl
              << "joint values are in [rad]." << std::endl
              << "motion-time is in [sec]." << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}
    );
    robot.setJointImpedance({{200.0, 200.0, 200.0, 200.0, 75.0, 50.0, 15.0}});

    JointPositionController motion_generator;
    std::array<double, 7> q_goal;
    double time_goal;

    if (argc >= 9) {
      for (size_t i = 0; i < 7; i++) {
        q_goal[i] = std::stod(argv[i + 2]);
      }
      JointPositionController motion_generator_g(q_goal);
      motion_generator = motion_generator_g;

      if (argc == 10) {
        time_goal = std::stod(argv[9]);
        JointPositionController motion_generator_gt(q_goal, time_goal);
        motion_generator = motion_generator_gt;
      }
    }

    robot.control(motion_generator);
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
