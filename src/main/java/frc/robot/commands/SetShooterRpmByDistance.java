package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.LookupMode;
import frc.robot.subsystems.ShooterSubsystem.SetpointMode;

public class SetShooterRpmByDistance extends Command {
  // One-shot command that selects RPM or Voltage based on distance, then ends.
  private final ShooterSubsystem shooter;
  private final double distanceMeters;
  private final SetpointMode mode;
  private final LookupMode lookupMode;

  public SetShooterRpmByDistance(ShooterSubsystem shooter, double distanceMeters, SetpointMode mode) {
    this(shooter, distanceMeters, mode, LookupMode.NEAREST);
  }

  public SetShooterRpmByDistance(
      ShooterSubsystem shooter,
      double distanceMeters,
      SetpointMode mode,
      LookupMode lookupMode) {
    this.shooter = shooter;
    this.distanceMeters = distanceMeters;
    this.mode = mode;
    this.lookupMode = lookupMode;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setTargetByDistance(distanceMeters, mode, lookupMode);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
