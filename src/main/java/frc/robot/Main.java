// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  // Utility class: not meant to be instantiated.
  private Main() {}

  // Program entry point on the roboRIO.
  // startRobot() creates the Robot instance and runs the main control loop.
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
