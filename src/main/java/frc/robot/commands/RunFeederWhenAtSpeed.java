package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Runs the feeder motor only when the shooter flywheel is at speed.
 * Intended for gated feed control (e.g., while a button is held).
 */
public class RunFeederWhenAtSpeed extends Command {
  private final ShooterSubsystem shooter;
  private final double volts;

  public RunFeederWhenAtSpeed(ShooterSubsystem shooter, double volts) {
    this.shooter = shooter;
    this.volts = volts;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if (shooter.isAtSpeed()) {
      shooter.runFeedVoltage(volts);
    } else {
      shooter.stopFeed();
    }
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
