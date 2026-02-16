// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * ShooterSubsystem - Controls dual-motor flywheel shooter with feed motor.
 * 
 * RECENT IMPROVEMENTS (Your Updates):
 * - isAtSpeed() method - Check if shooter is ready (EXCELLENT ADDITION!)
 * - Idle mode configuration - Explicit brake/coast settings
 * - "Shooter Ready" telemetry - Dashboard shows when ready to shoot
 * 
 * (See previous comments for full hardware architecture and control modes)
 */
public class ShooterSubsystem extends SubsystemBase {
  
  public enum SetpointMode {
    RPM,      // Closed-loop velocity control
    VOLTAGE   // Open-loop voltage control
  }
  
  public enum LookupMode {
    NEAREST,      // Snap to closest table entry
    INTERPOLATED  // Linear interpolation between entries
  }

  // ========================================================================================
  // HARDWARE INSTANCES
  // ========================================================================================
  
  private final SparkMax leader =
      new SparkMax(Constants.Shooter.kLeaderId, MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(Constants.Shooter.kFollowerId, MotorType.kBrushless);
  private final SparkMax feedMotor =
      new SparkMax(Constants.Shooter.kFeedId, MotorType.kBrushless);
  
  private final RelativeEncoder leaderEncoder = leader.getEncoder();
  private final RelativeEncoder followerEncoder = follower.getEncoder();
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Shooter.kLimitSwitchPort);
  private final Debouncer debouncer = 
      new Debouncer(Constants.Shooter.kDebounceTime, Debouncer.DebounceType.kRising);
  
  private final SparkClosedLoopController pid = leader.getClosedLoopController();
  
  // ========================================================================================
  // TELEMETRY & LOGGING
  // ========================================================================================
  
  private final DoubleLogEntry targetRpmLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/shooter/target_rpm");
  private final DoubleLogEntry targetVoltageLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/shooter/target_voltage");
  private final DoubleLogEntry leaderRpmLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/shooter/leader_rpm");
  private final DoubleLogEntry followerRpmLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/shooter/follower_rpm");
  
  private final GenericEntry leaderRpmEntry;
  private final GenericEntry followerRpmEntry;

  // ========================================================================================
  // STATE VARIABLES
  // ========================================================================================
  
  private double targetRpm = 0.0;
  private double targetVoltage = 0.0;
  private SetpointMode lastSetpointMode = SetpointMode.RPM;
  private double lastLoggedTargetRpm = Double.NaN;
  private double lastLoggedTargetVoltage = Double.NaN;
  
  SparkMaxConfig baseConfig = new SparkMaxConfig();

  // ========================================================================================
  // LOOKUP TABLES
  // ========================================================================================
  
  private static final double[] DISTANCE_TABLE_METERS = {
    Constants.Shooter.kLowDistanceMeters,
    Constants.Shooter.kMidDistanceMeters,
    Constants.Shooter.kHighDistanceMeters
  };
  
  private static final double[] RPM_TABLE = {
    Constants.Shooter.kLowRpm,
    Constants.Shooter.kMidRpm,
    Constants.Shooter.kHighRpm
  };
  
  private static final double[] VOLTAGE_TABLE = {
    Constants.Shooter.kLowVoltage,
    Constants.Shooter.kMidVoltage,
    Constants.Shooter.kHighVoltage
  };

  // ========================================================================================
  // CONSTRUCTOR
  // ========================================================================================
  
  public ShooterSubsystem() {
    // Shuffleboard graph widgets
    leaderRpmEntry =
        Shuffleboard.getTab("Shooter")
            .add("Shooter Leader RPM", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
    followerRpmEntry =
        Shuffleboard.getTab("Shooter")
            .add("Shooter Follower RPM", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

    // --------------------------------------------------------------------------------------
    // BASE CONFIGURATION
    // --------------------------------------------------------------------------------------
    baseConfig
        .inverted(true)
        
        /**
         * YOUR IMPROVEMENT: Explicit idle mode configuration
         * 
         * IDLE MODE SETTING - New addition to base config!
         * Previously this was left to Spark MAX defaults.
         * Now explicitly configured for predictable behavior.
         * 
         * CONFIGURED VALUE: Coast mode (see Constants.kShooterIdleMode)
         * 
         * WHY THIS MATTERS:
         * - Spark MAX default is BRAKE mode
         * - If not explicitly set, behavior could change between code versions
         * - Explicit configuration = predictable behavior
         * 
         * COAST MODE BENEFITS FOR SHOOTERS:
         * - Flywheel spins down naturally (lower mechanical stress)
         * - No current spike when stopping
         * - Quieter operation
         * - Better for high-speed flywheels
         * 
         * VERIFYING MODE ON ROBOT:
         * 1. Deploy code with Coast mode
         * 2. Spin up shooter to 3000 RPM
         * 3. Stop shooter (press X button)
         * 4. Observe: Wheel should coast down over 2-5 seconds
         * 
         * COMPARE WITH BRAKE MODE:
         * 1. Change Constants.kShooterIdleMode to IdleMode.kBrake
         * 2. Repeat test
         * 3. Observe: Wheel should stop in <1 second
         * 4. Listen for any harsh mechanical sounds (indicates too aggressive)
         * 5. Check current draw (may spike during electromagnetic braking)
         */
        .idleMode(Constants.Shooter.kShooterIdleMode)
        
        .smartCurrentLimit(Constants.Shooter.kSmartCurrentLimit)
        .openLoopRampRate(Constants.Shooter.kOpenLoopRampRate)
        .closedLoopRampRate(Constants.Shooter.kClosedLoopRampRate)
        .closedLoop
        .pid(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD)
        .iZone(Constants.Shooter.kIZone)
        .outputRange(-1.0, 1.0);
    
    baseConfig.closedLoop.feedForward.kV(Constants.Shooter.kFF);

    // --------------------------------------------------------------------------------------
    // LEADER CONFIGURATION
    // --------------------------------------------------------------------------------------
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.apply(baseConfig);
    leaderConfig.inverted(false);
    leader.configure(leaderConfig, 
                     ResetMode.kResetSafeParameters, 
                     PersistMode.kNoPersistParameters);

    // --------------------------------------------------------------------------------------
    // FOLLOWER CONFIGURATION
    // --------------------------------------------------------------------------------------
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.apply(baseConfig).follow(leader, Constants.Shooter.kFollowerInverted);
    follower.configure(followerConfig, 
                      ResetMode.kResetSafeParameters, 
                      PersistMode.kNoPersistParameters);

    // --------------------------------------------------------------------------------------
    // FEED MOTOR CONFIGURATION
    // --------------------------------------------------------------------------------------
    /**
     * YOUR IMPROVEMENT: Feed motor now has explicit idle mode too!
     * 
     * FEED MOTOR IDLE MODE:
     * - Set to Coast (same as shooter wheels)
     * - Feed motor doesn't need to hold position
     * - Coast mode is gentler on mechanism
     * 
     * ALTERNATIVE: Could use Brake mode if:
     * - Need instant stop when releasing feed button
     * - Game pieces tend to jam when motor coasts
     * - Want more aggressive feed control
     */
    SparkMaxConfig feedConfig = new SparkMaxConfig();
    feedConfig
        .smartCurrentLimit(Constants.Shooter.kSmartCurrentLimit)
        .openLoopRampRate(Constants.Shooter.kOpenLoopRampRate)
        .idleMode(Constants.Shooter.kFeedIdleMode)  // <- Idle mode now explicit!
        .inverted(false);
    feedMotor.configure(feedConfig, 
                        ResetMode.kResetSafeParameters, 
                        PersistMode.kNoPersistParameters);
  }

  // ========================================================================================
  // PUBLIC CONTROL METHODS
  // ========================================================================================

  public void setTargetRpm(double rpm) {
    targetRpm = MathUtil.clamp(rpm, 0.0, Constants.Shooter.kMaxRpm_Motor);
    targetVoltage = 0.0;
    lastSetpointMode = SetpointMode.RPM;
    pid.setSetpoint(targetRpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0);
  }

  public void setTargetByDistance(double distanceMeters, SetpointMode mode) {
    setTargetByDistance(distanceMeters, mode, LookupMode.NEAREST);
  }

  public void setTargetByDistance(double distanceMeters, SetpointMode mode, LookupMode lookupMode) {
    if (lookupMode == LookupMode.INTERPOLATED) {
      if (mode == SetpointMode.RPM) {
        setTargetRpm(getInterpolatedValue(distanceMeters, RPM_TABLE));
      } else {
        setTargetVoltage(getInterpolatedValue(distanceMeters, VOLTAGE_TABLE));
      }
      return;
    }

    int index = getNearestIndex(distanceMeters);
    if (mode == SetpointMode.RPM) {
      setTargetRpm(RPM_TABLE[index]);
    } else {
      setTargetVoltage(VOLTAGE_TABLE[index]);
    }
  }

  public void setTargetVoltage(double volts) {
    targetVoltage = MathUtil.clamp(volts, 0.0, Constants.Shooter.kMaxVoltage);
    targetRpm = 0.0;
    lastSetpointMode = SetpointMode.VOLTAGE;
    leader.setVoltage(targetVoltage);
  }

  public void stop() {
    targetRpm = 0.0;
    targetVoltage = 0.0;
    leader.stopMotor();
    feedMotor.stopMotor();
  }

  public void runFeedVoltage(double volts) {
    feedMotor.setVoltage(MathUtil.clamp(volts, 0.0, Constants.Shooter.kMaxVoltage));
  }

  public void stopFeed() {
    feedMotor.stopMotor();
  }

  // ========================================================================================
  // PUBLIC QUERY METHODS
  // ========================================================================================

  public double getTargetRpm() {
    return targetRpm;
  }

  public double getMeasuredRpm() {
    return leaderEncoder.getVelocity();
  }

  public double getFollowerMeasuredRpm() {
    return followerEncoder.getVelocity();
  }

  public boolean isLimitSwitchPressed() {
    return debouncer.calculate(limitSwitch.get());
  }

  /**
   * YOUR IMPROVEMENT: isAtSpeed() method - Check if shooter is ready to fire!
   * 
   * EXCELLENT ADDITION! This is exactly what I recommended in the code review.
   * This method answers the critical question: "Is it safe to shoot now?"
   * 
   * WHAT IT DOES:
   * Returns true when shooter is:
   * 1. In RPM mode (not voltage mode)
   * 2. Has non-zero target (not stopped)
   * 3. Within tolerance of target speed (+/-50 RPM default)
   * 
   * WHY THIS IS IMPORTANT:
   * 
   * WITHOUT isAtSpeed():
   * - Operator presses "shoot" button
   * - Shooter might still be spinning up (3000 RPM target, currently at 2500 RPM)
   * - Game piece shoots too slow
   * - Shot misses target
   * - Inconsistent performance
   * 
   * WITH isAtSpeed():
   * - Operator presses "prepare to shoot" button
   * - Dashboard shows "Shooter Ready: false" (red light)
   * - Shooter spins up to 3000 RPM
   * - When within +/-50 RPM: "Shooter Ready: true" (green light)
   * - Operator presses "shoot" button (or auto-command proceeds)
   * - Consistent shot velocity every time!
   * 
   * @return true if shooter is ready to fire, false otherwise
   * 
   * METHOD BREAKDOWN:
   */
  public boolean isAtSpeed() {
    /**
     * CONDITION 1: Must be in RPM mode
     * 
     * WHY CHECK MODE?
     * - In VOLTAGE mode, we don't have a specific speed target
     * - Voltage mode is open-loop (no speed feedback)
     * - Can't determine "at speed" without a speed target
     * 
     * BEHAVIOR:
     * - RPM mode: Check if at target speed
     * - VOLTAGE mode: Always return false (speed unknown)
     * 
     * ALTERNATIVE DESIGN:
     * Could return true in voltage mode after time delay:
     *   if (lastSetpointMode == SetpointMode.VOLTAGE) {
     *     return (currentTime - voltageSetTime) > 0.5; // 500ms spinup
     *   }
     */
    if (lastSetpointMode != SetpointMode.RPM || targetRpm <= 0.0) {
      return false;  // Not in RPM mode or shooter is stopped
    }
    
    /**
     * CONDITION 2: Within tolerance of target
     * 
     * THE CORE CHECK:
     * Math.abs(getMeasuredRpm() - targetRpm) <= Constants.Shooter.kRpmTolerance
     * 
     * BREAKDOWN:
     * 1. getMeasuredRpm() - targetRpm = error in RPM
     *    Example: 5000 - 4960 = 40 RPM error
     * 
     * 2. Math.abs(...) = absolute value (ignore sign)
     *    Example: abs(40) = 40
     *    Example: abs(-40) = 40
     *    We don't care if over or under target, just how far off
     * 
     * 3. error <= kRpmTolerance = within acceptable range?
     *    Example: 40 <= 50 -> true (within tolerance)
     *    Example: 60 <= 50 -> false (outside tolerance)
     * 
     * EXAMPLE SCENARIOS:
     * 
     * Scenario 1: Perfect speed
     * - Target: 5000 RPM
     * - Actual: 5000 RPM
     * - Error: |5000 - 5000| = 0 RPM
     * - Result: 0 <= 50 -> true OK
     * 
     * Scenario 2: Within tolerance (slightly low)
     * - Target: 5000 RPM
     * - Actual: 4960 RPM
     * - Error: |4960 - 5000| = 40 RPM
     * - Result: 40 <= 50 -> true OK
     * 
     * Scenario 3: Within tolerance (slightly high)
     * - Target: 5000 RPM
     * - Actual: 5040 RPM
     * - Error: |5040 - 5000| = 40 RPM
     * - Result: 40 <= 50 -> true OK
     * 
     * Scenario 4: Outside tolerance
     * - Target: 5000 RPM
     * - Actual: 4920 RPM
     * - Error: |4920 - 5000| = 80 RPM
     * - Result: 80 <= 50 -> false NO
     * 
     * TUNING kRpmTolerance:
     * See Constants.java for detailed tuning guidance.
     * Current value: 50 RPM (good starting point)
     */
    return Math.abs(getMeasuredRpm() - targetRpm) <= Constants.Shooter.kRpmTolerance;
  }
  
  /**
   * USAGE EXAMPLES:
   * 
   * Example 1: Wait for ready in autonomous
   * ```java
   * // In autonomous command
   * shooter.setTargetRpm(5000);
   * 
   * // Wait until ready (blocking - not recommended for long waits)
   * while (!shooter.isAtSpeed()) {
   *   // Busy wait (ok for <1 second)
   * }
   * 
   * // Now safe to shoot
   * shooter.runFeedVoltage(6.0);
   * ```
   * 
   * Example 2: Better pattern with timeout
   * ```java
   * new SetShooterRpm(shooter, 5000)
   *   .andThen(new WaitUntilCommand(shooter::isAtSpeed)
   *     .withTimeout(2.0))  // Give up after 2 seconds
   *   .andThen(new FeedGamePiece(shooter));
   * ```
   * 
   * Example 3: Conditional auto-shoot
   * ```java
   * // In execute() method of auto-shoot command
   * if (shooter.isAtSpeed() && vision.hasTarget()) {
   *   shooter.runFeedVoltage(6.0);
   * }
   * ```
   * 
   * Example 4: Dashboard indicator
   * ```java
   * // Already done in publishTelemetry()!
   * SmartDashboard.putBoolean("Shooter Ready", isAtSpeed());
   * ```
   * 
   * ADVANCED USAGE - LED Indicator:
   * ```java
   * // In periodic() or LED subsystem
   * if (shooter.isAtSpeed()) {
   *   led.setColor(Color.kGreen);  // Green = ready to shoot
   * } else if (shooter.getTargetRpm() > 0) {
   *   led.setColor(Color.kYellow); // Yellow = spinning up
   * } else {
   *   led.setColor(Color.kRed);    // Red = stopped
   * }
   * ```
   */

  // ========================================================================================
  // PERIODIC METHOD
  // ========================================================================================

  @Override
  public void periodic() {
    publishTelemetry();
  }

  /**
   * YOUR IMPROVEMENT: Added "Shooter Ready" to telemetry!
   * 
   * This is line 263:
   * SmartDashboard.putBoolean("Shooter Ready", isAtSpeed());
   * 
   * BENEFITS:
   * - Driver gets instant visual feedback
   * - Can wait for green indicator before shooting
   * - Dramatically improves shot consistency
   * - Easy to debug spinup issues
   * 
   * DASHBOARD DISPLAY OPTIONS:
   * 
   * Option 1: Boolean Box (default)
   * - Shows true/false text
   * - Changes color (usually green/red)
   * 
   * Option 2: LED Widget
   * - Visual indicator light
   * - More obvious for operators
   * - Can customize colors
   * 
   * Option 3: Custom Widget
   * - Show RPM value + ready state
   * - "5000 RPM OK READY" vs "4960 RPM SPINNING SPINNING UP"
   * 
   * SHUFFLEBOARD ENHANCEMENT:
   * ```java
   * Shuffleboard.getTab("Shooter")
   *   .addBoolean("Ready to Shoot", this::isAtSpeed)
   *   .withWidget(BuiltInWidgets.kBooleanBox)
   *   .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "red"));
   * ```
   */
  private void publishTelemetry() {
    SmartDashboard.putNumber("Shooter Target RPM", targetRpm);
    SmartDashboard.putNumber("Shooter Target Voltage", targetVoltage);
    SmartDashboard.putString("Shooter Setpoint Mode", lastSetpointMode.name());
    SmartDashboard.putNumber("Shooter Leader RPM", getMeasuredRpm());
    SmartDashboard.putNumber("Shooter Follower RPM", getFollowerMeasuredRpm());
    
    /**
     * "Shooter Ready" telemetry - NEW LINE!
     * Shows green when safe to shoot, red when still spinning up.
     */
    SmartDashboard.putBoolean("Shooter Ready", isAtSpeed());
    
    SmartDashboard.putBoolean("Shooter Limit Switch", isLimitSwitchPressed());
    SmartDashboard.putNumber("Shooter Leader output", leader.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Follower output", follower.getAppliedOutput());
    SmartDashboard.putNumber("leader bus voltage", leader.getBusVoltage());
    SmartDashboard.putNumber("follower bus voltage", follower.getBusVoltage());
    SmartDashboard.putNumber("Shooter kFF", Constants.Shooter.kFF);
    SmartDashboard.putNumber("Shooter kP", Constants.Shooter.kP);

    // DataLog for post-run analysis
    if (Double.isNaN(lastLoggedTargetRpm)
        || Math.abs(targetRpm - lastLoggedTargetRpm) > Constants.Shooter.kLogEpsilon) {
      targetRpmLog.append(targetRpm);
      lastLoggedTargetRpm = targetRpm;
    }
    if (Double.isNaN(lastLoggedTargetVoltage)
        || Math.abs(targetVoltage - lastLoggedTargetVoltage) > Constants.Shooter.kLogEpsilon) {
      targetVoltageLog.append(targetVoltage);
      lastLoggedTargetVoltage = targetVoltage;
    }
    leaderRpmLog.append(getMeasuredRpm());
    followerRpmLog.append(getFollowerMeasuredRpm());
    leaderRpmEntry.setDouble(getMeasuredRpm());
    followerRpmEntry.setDouble(getFollowerMeasuredRpm());
  }

  // ========================================================================================
  // PRIVATE HELPER METHODS
  // ========================================================================================

  private int getNearestIndex(double distanceMeters) {
    int bestIndex = 0;
    double bestDistance = Math.abs(distanceMeters - DISTANCE_TABLE_METERS[0]);
    for (int i = 1; i < DISTANCE_TABLE_METERS.length; i++) {
      double delta = Math.abs(distanceMeters - DISTANCE_TABLE_METERS[i]);
      if (delta < bestDistance) {
        bestDistance = delta;
        bestIndex = i;
      }
    }
    return bestIndex;
  }

  private double getInterpolatedValue(double distanceMeters, double[] values) {
    if (DISTANCE_TABLE_METERS.length != values.length) {
      return values[0];
    }
    if (distanceMeters <= DISTANCE_TABLE_METERS[0]) {
      return values[0];
    }
    int last = DISTANCE_TABLE_METERS.length - 1;
    if (distanceMeters >= DISTANCE_TABLE_METERS[last]) {
      return values[last];
    }

    for (int i = 0; i < last; i++) {
      double d0 = DISTANCE_TABLE_METERS[i];
      double d1 = DISTANCE_TABLE_METERS[i + 1];
      if (distanceMeters >= d0 && distanceMeters <= d1) {
        double t = (distanceMeters - d0) / (d1 - d0);
        return values[i] + t * (values[i + 1] - values[i]);
      }
    }
    return values[last];
  }
}

