package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Runs the feeder motor at a fixed voltage while scheduled.
 * Intended for manual control (e.g., while a button is held).
 */
public class RunFeederManual extends Command {
  private final ShooterSubsystem shooter;
  private final double volts;

  public RunFeederManual(ShooterSubsystem shooter, double volts) {
    this.shooter = shooter;
    this.volts = volts;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.runFeedVoltage(volts);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFeed();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
