// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Constants - Central repository for all robot-wide constant values.
 * 
 * RECENT IMPROVEMENTS (Your Updates):
 * ✅ Motor type profile enum for easy NEO/Vortex switching
 * ✅ Automatic current limit adjustment based on motor type
 * ✅ Idle mode configuration (brake vs coast)
 * ✅ RPM tolerance constant for isAtSpeed() checking
 * 
 * WHY CENTRALIZE CONSTANTS?
 * 1. Single Source of Truth: One place to find all tunable values
 * 2. Easy Tuning: Change CAN ID or PID gain without searching through code
 * 3. Prevent Magic Numbers: Named constants are self-documenting
 * 4. Type Safety: final prevents accidental modification
 * 5. Organization: Inner classes group related constants
 */
/*
This file is the single source of truth for tunable robot values.
It groups constants by subsystem to make tuning and hardware changes
predictable and easy to review.

Constants overview:
1. centralizes CAN IDs, setpoints, and limits,
2. reduces magic numbers across the codebase,
3. makes tuning changes safe and traceable.
*/
public final class Constants {
  /**
   * Private constructor prevents instantiation.
   * Constants is a utility class (only static members).
   */
  private Constants() {}

  // ========================================================================================
  // OPERATOR INTERFACE - Controller and input device configuration
  // ========================================================================================
  
  public static final class OI {
    /**
     * USB port for driver Xbox controller (always present).
     * Valid range: 0-5 (six USB ports on Driver Station)
     */
    public static final int kDriverPort = 0;

    /**
     * USB port for non-driver controls (operator Xbox or keypad).
     */
    public static final int kOperatorPort = 1;

    /**
     * USB port for optional keypad/button panel.
     * Same port as operator Xbox when only one non-driver device is used.
     */
    public static final int kKeypadPort = 1;

    /**
     * Control selection:
     * - Default to operator Xbox (true) for non-driver controls.
     * - Enable keypad bindings if using the keypad instead (or later, both).
     */
    public static final boolean kEnableOperatorXbox = true;
    public static final boolean kEnableKeypadBindings = false;

    private OI() {}
  }

  // ========================================================================================
  // SHOOTER - All shooter subsystem configuration
  // ========================================================================================
  
  public static final class Shooter {
    
    // ------------------------------------------------------------------------------------
    // MOTOR TYPE SELECTION - New smart profile system!
    // ------------------------------------------------------------------------------------
    
    /**
     * MotorTypeProfile - Enum for motor type selection.
     * 
     * YOUR IMPROVEMENT:
     * This is an excellent pattern! Instead of manually changing multiple constants
     * when swapping motors, you just change one enum value and everything updates.
     * 
     * BENEFITS:
     * - Single point of change (set kMotorTypeProfile once)
     * - Automatic adjustments to maxRPM, current limits, feedforward
     * - Prevents mistakes (forgetting to update one constant)
     * - Self-documenting (code clearly shows motor type)
     * 
     * MOTOR COMPARISON:
     * NEO (REV-21-1650):
     * - Free Speed: 5676 RPM
     * - Stall Current: 105A
     * - Continuous: ~20-30A
     * - Weight: 0.94 lbs
     * - Use: General purpose, proven reliable
     * 
     * VORTEX (REV-21-1652):
     * - Free Speed: 6784 RPM (+20% faster)
     * - Stall Current: 211A
     * - Continuous: ~40-60A
     * - Weight: 1.14 lbs
     * - Use: High performance, shooters, heavy loads
     * - Released: 2024 season
     */
    public enum MotorTypeProfile {
      NEO,      // Standard NEO brushless motor
      VORTEX    // High-performance NEO Vortex motor
    }

    /**
     * ACTIVE MOTOR SELECTION - Change here to switch motor types!
     * 
     * CURRENT: NEO motors
     * 
     * TO SWITCH TO VORTEX:
     * 1. Change this line to: kMotorTypeProfile = MotorTypeProfile.VORTEX
     * 2. That's it! Current limits and max RPM update automatically
     * 3. May need to re-tune PID (different motor dynamics)
     * 
     * WHAT UPDATES AUTOMATICALLY:
     * - kMaxRpm_Motor (line 72)
     * - kSmartCurrentLimit (line 90-91)
     * - kFF (feedforward, line 86 - calculated from max RPM)
     * 
     * WHAT NEEDS MANUAL TUNING:
     * - PID gains (kP, kI, kD) - different inertia and friction
     * - May need to adjust ramp rates
     * - Test all presets on actual hardware
     */
    public static final MotorTypeProfile kMotorTypeProfile = MotorTypeProfile.NEO;

    // ------------------------------------------------------------------------------------
    // HARDWARE CONFIGURATION - CAN IDs and motor setup
    // ------------------------------------------------------------------------------------
    
    public static final int kLeaderId = 27;      // Leader motor CAN ID
    public static final int kFollowerId = 23;    // Follower motor CAN ID
    public static final int kFeedId = 28;        // Feed motor CAN ID
    public static final boolean kFollowerInverted = true;  // Follower output inversion

    // ------------------------------------------------------------------------------------
    // RPM PRESETS - Discrete velocity setpoints
    // ------------------------------------------------------------------------------------
    
    public static final double kVeryLowRpm = 1500.0;   // Testing/very close shots
    public static final double kLowRpm = 2000.0;       // Close range shots
    public static final double kMidRpm = 3250.0;       // Mid-range shots
    public static final double kHighRpm = 5700.0;      // Long range shots
    public static final double kVeryHighRpm = 6200.0;  // Maximum power shots
    
    // ------------------------------------------------------------------------------------
    // DISTANCE LOOKUP TABLES
    // ------------------------------------------------------------------------------------
    
    public static final double kLowDistanceMeters = 5.0;   // Near reference point
    public static final double kMidDistanceMeters = 8.0;   // Mid reference point
    public static final double kHighDistanceMeters = 10.0; // Far reference point
    
    // Distance presets for button bindings
    public static final double kPresetDistanceVeryNearMeters = 2.0;
    public static final double kPresetDistanceNearMeters = 3.0;
    public static final double kPresetDistanceMidMeters = 6.0;
    public static final double kPresetDistanceFarMeters = 9.0;
    public static final double kPresetDistanceVeryFarMeters = 12.0;
    
    // ------------------------------------------------------------------------------------
    // VOLTAGE PRESETS - Open-loop control
    // ------------------------------------------------------------------------------------
    
    public static final double kLowVoltage = 6.0;    // Low power
    public static final double kMidVoltage = 8.0;    // Medium power
    public static final double kHighVoltage = 10.0;  // High power
    public static final double kFeedVoltage = 6.0;   // Feed motor voltage
    public static final double kMaxVoltage = 12.0;   // Maximum allowed voltage

    // ------------------------------------------------------------------------------------
    // MOTOR SPECIFICATIONS - Automatic selection based on profile
    // ------------------------------------------------------------------------------------
    
    /**
     * Maximum free speed for each motor type.
     * These are manufacturer specifications at 12V.
     */
    public static final double kMaxRpm_SparkMaxNeo = 5676.0;   // NEO free speed
    public static final double kMaxRpm_FlexVortex = 6784.0;    // Vortex free speed
    
    /**
     * AUTOMATIC MAX RPM SELECTION - Updates based on kMotorTypeProfile
     * 
     * YOUR IMPROVEMENT:
     * This ternary operator automatically picks the right max RPM.
     * 
     * HOW IT WORKS:
     * (condition) ? valueIfTrue : valueIfFalse
     * 
     * If motor is NEO → use 5676 RPM
     * If motor is VORTEX → use 6784 RPM
     * 
     * JAVA TERNARY OPERATOR:
     * condition ? trueValue : falseValue
     * 
     * EQUIVALENT IF-ELSE:
     * if (kMotorTypeProfile == MotorTypeProfile.NEO) {
     *   kMaxRpm_Motor = kMaxRpm_SparkMaxNeo;
     * } else {
     *   kMaxRpm_Motor = kMaxRpm_FlexVortex;
     * }
     * 
     * But ternary is more concise and works with final variables!
     */
    public static final double kMaxRpm_Motor =
        (kMotorTypeProfile == MotorTypeProfile.NEO) ? kMaxRpm_SparkMaxNeo : kMaxRpm_FlexVortex;

    // ------------------------------------------------------------------------------------
    // PID + FEEDFORWARD GAINS
    // ------------------------------------------------------------------------------------
    
    public static final double kP = 0.00018;   // Proportional gain
    public static final double kI = 0.0;       // Integral gain (disabled)
    public static final double kD = 0.00000;   // Derivative gain (disabled)
    
    /**
     * FEEDFORWARD CALCULATION - Automatic based on motor max RPM
     * 
     * FORMULA: kFF = Vmax / ωmax
     * - Vmax: Battery voltage (12V nominal)
     * - ωmax: Motor free speed (RPM)
     * 
     * AUTOMATIC UPDATES:
     * - NEO: 12.0 / 5676 ≈ 0.00211 volts/RPM
     * - Vortex: 12.0 / 6784 ≈ 0.00177 volts/RPM
     * 
     * WHY DIFFERENT?
     * Vortex spins faster at same voltage, so needs less voltage per RPM.
     */
    public static final double kFF = 12.0 / kMaxRpm_Motor;
    
    public static final double kIZone = 0.0;   // Integral zone (disabled)
    
    // ------------------------------------------------------------------------------------
    // CURRENT LIMITING - Automatic adjustment based on motor type
    // ------------------------------------------------------------------------------------
    
    /**
     * SMART CURRENT LIMIT - Motor-specific safe operating current
     * 
     * YOUR IMPROVEMENT:
     * Automatically adjusts current limit based on motor type!
     * 
     * NEO CURRENT LIMIT: 40A
     * - Still at stall current (not ideal)
     * - RECOMMENDATION: Reduce to 30A for safer continuous operation
     * - NEO specs: 20-30A continuous, 105A stall
     * 
     * VORTEX CURRENT LIMIT: 50A
     * - More conservative than stall (211A)
     * - Good starting point for testing
     * - Vortex can handle higher continuous current than NEO
     * - May increase to 60A after testing if needed
     * 
     * TESTING PROCEDURE:
     * 1. Monitor actual current draw in SmartDashboard
     * 2. Run shooter under full load (with game pieces)
     * 3. Check for current limiting (output bounces/fluctuates)
     * 4. Adjust up if limiting prevents full speed
     * 5. Adjust down if motors run hot (>80°C)
     * 
     * TEMPERATURE MONITORING:
     * Spark MAX can report motor temperature via getMotorTemperature()
     * Consider adding temperature telemetry for safety monitoring.
     */
    public static final int kSmartCurrentLimit =
        (kMotorTypeProfile == MotorTypeProfile.NEO) ? 40 : 50;
    
    // ------------------------------------------------------------------------------------
    // RAMP RATES
    // ------------------------------------------------------------------------------------
    
    public static final double kOpenLoopRampRate = 0.1;      // Open-loop ramp (seconds)
    public static final double kClosedLoopRampRate = 0.0;    // Closed-loop ramp (disabled)
    
    // ------------------------------------------------------------------------------------
    // IDLE MODE CONFIGURATION - New addition!
    // ------------------------------------------------------------------------------------
    
    /**
     * IDLE MODE - Brake vs Coast behavior when motor output is zero
     * 
     * YOUR IMPROVEMENT:
     * Explicitly configured idle modes for shooter and feed motors!
     * 
     * BRAKE MODE (IdleMode.kBrake):
     * - Motor terminals shorted together
     * - Creates electromagnetic braking force
     * - Motor stops quickly
     * - Holds position against external forces
     * - Higher current draw during deceleration
     * 
     * COAST MODE (IdleMode.kCoast):
     * - Motor terminals open (high impedance)
     * - No braking force applied
     * - Motor coasts to stop based on friction
     * - Spins freely when pushed
     * - Lower current draw
     * 
     * CURRENT SETTINGS: Both set to COAST
     * 
     * WHY COAST FOR SHOOTERS?
     * ✅ Less mechanical stress (no abrupt stops)
     * ✅ Lower current draw
     * ✅ Allows flywheel to coast down naturally
     * ✅ Reduces gear wear
     * 
     * WHEN TO USE BRAKE:
     * - Drivetrains (prevents robot from rolling)
     * - Elevators (holds position under load)
     * - Arms (prevents sagging)
     * - Any mechanism that needs position holding
     * 
     * TESTING BOTH MODES:
     * Try switching to kBrake and observe:
     * - Does shooter stop faster? (usually yes)
     * - Any harsh mechanical sounds? (potential issue)
     * - Current spikes during stop? (check SmartDashboard)
     * - Does it affect shot consistency? (test empirically)
     */
    public static final IdleMode kShooterIdleMode = IdleMode.kCoast;  // Shooter wheels
    public static final IdleMode kFeedIdleMode = IdleMode.kCoast;     // Feed motor

    // ------------------------------------------------------------------------------------
    // LIMIT SWITCH & SENSING
    // ------------------------------------------------------------------------------------
    
    public static final int kLimitSwitchPort = 0;          // DIO port for limit switch
    public static final double kDebounceTime = 0.02;        // Debounce duration (20ms)
    
    /**
     * RPM TOLERANCE - How close to target counts as "at speed"
     * 
     * CURRENT VALUE: 50 RPM
     * 
     * HOW IT'S USED:
     * In ShooterSubsystem.isAtSpeed():
     *   return Math.abs(getMeasuredRpm() - targetRpm) <= kRpmTolerance;
     * 
     * MEANING:
     * - Target: 5000 RPM
     * - Actual: 4960-5040 RPM → isAtSpeed() returns true
     * - Actual: 4940 RPM → isAtSpeed() returns false (off by 60)
     * 
     * TUNING GUIDANCE:
     * 
     * TOO TIGHT (< 20 RPM):
     * - May never reach "ready" state due to normal oscillation
     * - Delays shooting unnecessarily
     * - Watch for PID oscillation (±10-20 RPM is normal)
     * 
     * TOO LOOSE (> 100 RPM):
     * - Allows shooting before fully spun up
     * - Inconsistent shot velocity
     * - Reduced accuracy
     * 
     * SWEET SPOT (30-75 RPM):
     * - 50 RPM is a good starting point
     * - Accounts for normal PID settling
     * - Still ensures consistent shooting
     * 
     * TESTING PROCEDURE:
     * 1. Spin up shooter and monitor actual RPM on dashboard
     * 2. Observe how much it oscillates around target (±X RPM)
     * 3. Set tolerance to 2-3X the oscillation amplitude
     * 4. Verify shots are consistent when "ready" light is on
     * 
     * DASHBOARD USAGE:
     * Display "Shooter Ready" boolean on SmartDashboard
     * Operator can wait for green light before shooting
     * Improves shot consistency dramatically!
     */
    public static final double kRpmTolerance = 50.0;
    
    // ------------------------------------------------------------------------------------
    // TELEMETRY
    // ------------------------------------------------------------------------------------
    
    /**
     * Logging epsilon - Threshold for change detection in data logging.
     * Only log target values when they change by more than this amount.
     */
    public static final double kLogEpsilon = 1e-6;

    private Shooter() {}
  }
}
