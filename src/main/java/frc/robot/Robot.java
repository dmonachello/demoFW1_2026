// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/*
Main lifecycle class for the robot program.
It initializes logging and the container, and drives the command scheduler.

Lifecycle flow:
1. set up logging and RobotContainer,
2. run CommandScheduler every cycle,
3. handle mode transitions (auto/teleop/test/disabled).
*/
public class Robot extends TimedRobot {
  // Holds the autonomous command selected by RobotContainer.
  private Command m_autonomousCommand;

  // RobotContainer wires subsystems, commands, and button bindings together.
  private final RobotContainer m_robotContainer;

  public Robot() {
    // Start WPILib data logging (useful for debugging after a match).
    DataLogManager.start();
    // Log NetworkTables too (dashboard values, etc).
    DataLogManager.logNetworkTables(true);
    // Build our container after logging is up.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler every 20 ms.
    // This drives command lifecycle: initialize/execute/end.
    CommandScheduler.getInstance().run();
  }

  @Override
  // Called once when the robot is disabled.
  public void disabledInit() {}

  @Override
  // Called every 20 ms while disabled.
  public void disabledPeriodic() {}

  @Override
  // Called once when leaving disabled mode.
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Ask RobotContainer which auto command to run.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule it if we have one.
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  // Called every 20 ms during autonomous.
  public void autonomousPeriodic() {}

  @Override
  // Called once when leaving autonomous.
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Cancel auto when teleop starts so drivers take control.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  // Called every 20 ms during teleop.
  public void teleopPeriodic() {}

  @Override
  // Called once when leaving teleop.
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Safety: stop all running commands when entering test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  // Called every 20 ms during test.
  public void testPeriodic() {}

  @Override
  // Called once when leaving test.
  public void testExit() {}
}
