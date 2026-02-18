// src/main/java/frc/robot/commands/RunShooterUntilStopped.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/*
Runs the shooter at a target RPM until interrupted.
Used for "hold to run" or persistent shooter spin.

Current bindings (RobotContainer):
1. onTrue (one-shot start binding used for persistent run).

Binding impact:
1. onTrue means it starts on press and runs until another shooter command interrupts,
2. if bound with whileTrue in the future, it would stop on button release.

Termination:
1. ends when interrupted or canceled by another command,
2. ends on button release when bound with whileTrue,
3. otherwise runs until explicitly canceled.
*/
public class RunShooterUntilStopped extends Command {
  // This command keeps the shooter running until it is interrupted/canceled.
  // Scheduler basics for students:
  // - The CommandScheduler runs every 20 ms.
  // - It tracks which commands "require" which subsystems.
  // - Only one command can require a given subsystem at a time.
  // - When a new command with the same requirement is scheduled, the old one is interrupted.
  //
  // This command requires the shooter subsystem while scheduled.
  private final ShooterSubsystem shooter;
  private final double rpm;

  // Constructor runs once when the command object is created.
  public RunShooterUntilStopped(ShooterSubsystem shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
    // Requirements prevent two commands from controlling the same subsystem at once.
    addRequirements(shooter);
  }

  // initialize() runs once when the command is scheduled.
  // There is no execute() override, so nothing repeats every 20 ms for this command.
  // It simply holds the requirement until interrupted.
  @Override
  public void initialize() {
    shooter.setTargetRpm(rpm);
  }

  // Called when the command ends. "interrupted" is true if another
  // command that requires ShooterSubsystem was scheduled or if a wrapper
  // like .until(...) canceled it.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Always false here, so the command runs until it is interrupted or canceled.
  @Override
  public boolean isFinished() {
    return false;
  }
}
