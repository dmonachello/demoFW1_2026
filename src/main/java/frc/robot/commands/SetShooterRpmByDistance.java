// Package groups commands so they are discoverable and organized by role.
package frc.robot.commands;

// Base command type from WPILib; provides lifecycle hooks and scheduler integration.
import edu.wpi.first.wpilibj2.command.Command;
// Subsystem dependency used to set the shooter target.
import frc.robot.subsystems.ShooterSubsystem;
// Enum that defines how a distance lookup should be chosen.
import frc.robot.subsystems.ShooterSubsystem.LookupMode;
// Enum that defines the kind of target to command (e.g., RPM vs voltage).
import frc.robot.subsystems.ShooterSubsystem.SetpointMode;

/*
This command only tells the shooter subsystem what target to use.
It does not run the control loop itself; the subsystem is expected
to already handle closed-loop control in its own periodic updates.

Current bindings (RobotContainer):
1. onTrue (one-shot setpoint binding used for distance presets).

Binding impact:
1. onTrue means it sets the target once and ends immediately,
2. if bound with whileTrue in the future, it would keep reissuing the target.

Termination:
1. ends immediately after initialize() (isFinished returns true),
2. interrupts/cancels are technically possible but not meaningful.

One-shot command:
1. resolves a shooter setpoint from distance,
2. issues the target,
3. and immediately finishes.
*/
public class SetShooterRpmByDistance extends Command {
  // Holds the subsystem reference so we can send it the target.
  private final ShooterSubsystem shooter;
  // Distance input used for the lookup table or model (meters) to pick a target.
  private final double distanceMeters;
  // Determines whether the target is expressed in RPM, voltage, or other modes
  // supported by the shooter subsystem, aligning with how the subsystem controls the motor.
  private final SetpointMode mode;
  // Controls how distance-to-setpoint lookup is performed (e.g., nearest or
  // interpolated values), which impacts accuracy vs simplicity.
  private final LookupMode lookupMode;

  // Convenience constructor: callers only specify distance and mode.
  public SetShooterRpmByDistance(ShooterSubsystem shooter, double distanceMeters, SetpointMode mode) {
    // Constructor chaining: delegate to the full constructor and supply a
    // default lookup mode (NEAREST) to avoid duplicating init logic here.
    this(shooter, distanceMeters, mode, LookupMode.NEAREST);
  }

  // Full constructor: lets callers pick the exact lookup behavior.
  public SetShooterRpmByDistance(
      ShooterSubsystem shooter,
      double distanceMeters,
      SetpointMode mode,
      LookupMode lookupMode) {
    // Store inputs; these are immutable for this command instance and define its behavior.
    this.shooter = shooter;
    // Persist the distance so initialize() can compute the target once.
    this.distanceMeters = distanceMeters;
    // Persist the setpoint mode to match the chosen control strategy.
    this.mode = mode;
    // Persist the lookup mode to match the desired distance-to-target selection.
    this.lookupMode = lookupMode;
    // Declare the shooter dependency so this command has exclusive access
    // while it runs (even though it completes immediately).
    addRequirements(shooter);
  }

  @Override
  // Called once when the command is scheduled, used to push the target to the subsystem.
  public void initialize() {
    // Issue the target once. The subsystem is responsible for any control loop,
    // ramping, or closed-loop enablement.
    shooter.setTargetByDistance(distanceMeters, mode, lookupMode);
  }

  @Override
  // Returning true makes this command end immediately after initialize().
  public boolean isFinished() {
    // Command is fire-and-forget; no need to wait for shooter to reach target.
    return true;
  }
}
