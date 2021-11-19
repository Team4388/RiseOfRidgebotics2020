/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum Mode {
    COMPETITIVE, CASUAL;

    private static Mode mode = Mode.COMPETITIVE;
    private static ArrayList<Consumer<Mode>> changeHandlers = new ArrayList<>();

    public static void register(Consumer<Mode> changeHandler) {
      changeHandlers.add(changeHandler);
    }

    public static Mode get() {
      return mode;
    }

    public static void set(Mode mode) {
      System.out.println(mode);
      int i = mode.ordinal();
      Mode.mode = mode;
      CommandScheduler.getInstance().disable();
      changeHandlers.forEach(c -> c.accept(mode));
      CommandScheduler.getInstance().enable();
      DriveConstants.DRIVE_WITH_JOYSTICK_FACTOR = DriveConstants.DRIVE_WITH_JOYSTICK_FACTOR_MODES[i];
      IntakeConstants.INTAKE_SPEED = IntakeConstants.INTAKE_SPEED_MODES[i];
      StorageConstants.STORAGE_SPEED = StorageConstants.STORAGE_SPEED_MODES[i];
    }

    public static void toggle() {
      int i = mode.ordinal() + 1;
      Mode[] values = values();
      i = i >= values.length ? 0 : i;
      set(values[i]);
    }
  }

  public static final int SELECTED_AUTO = 0;

  public static final class DriveConstants {
    /* Drive Train IDs */
    public static final int DRIVE_LEFT_FRONT_CAN_ID = 2;
    public static final int DRIVE_RIGHT_FRONT_CAN_ID = 4;
    public static final int DRIVE_LEFT_BACK_CAN_ID = 3;
    public static final int DRIVE_RIGHT_BACK_CAN_ID = 5;
    public static final int PIGEON_ID = 6;

    /* Drive Inversions */
    public static final boolean isRightMotorInverted = true;
    public static final boolean isLeftMotorInverted = false;
    public static final boolean isRightArcadeInverted = false;
    public static final boolean isAuxPIDInverted = false;

    /* Drive Configuration */
    public static final int DRIVE_TIMEOUT_MS = 30; // Use for all motor config
    public static final double OPEN_LOOP_RAMP_RATE = 0.2; // Seconds from 0 to full power on motor
    public static final double NEUTRAL_DEADBAND = 0.04;
    public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(false, 60, 40, 2);
    public static final int CLOSED_LOOP_TIME_MS = 1; // Higher numbers mean slower control loops
    public static final double COS_MULTIPLIER_LOW = 1.0;
    public static final double COS_MULTIPLIER_HIGH = 0.8;

    /* Drive Train Characteristics */
    public static final double MOTOR_ROT_PER_WHEEL_ROT_HIGH = 7.29;
    public static final double MOTOR_ROT_PER_WHEEL_ROT_LOW = 15;
    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double TICKS_PER_GYRO_REV = 8192;
    public static final double TICKS_PER_MOTOR_REV = 2048;

    /* PID Constants Drive*/
    public static final Gains DRIVE_DISTANCE_GAINS_LOW = new Gains(0.1, 0.0, 1.0, 0.0, 0, 0.5);
    public static final Gains DRIVE_VELOCITY_GAINS_LOW = new Gains(0.05, 0.0, 1.0, 0.025, 0, 1.0);
    public static final Gains DRIVE_TURNING_GAINS_LOW = new Gains(0.5, 0.0, 2.0, 0.0, 0, 0.55);
    public static final Gains DRIVE_MOTION_MAGIC_GAINS_LOW = new Gains(0.1, 0.0, 0, 0.025, 0, 1.0);
    public static final int DRIVE_CRUISE_VELOCITY = 30000;
    public static final int DRIVE_ACCELERATION = 23000;

    private static final double[] DRIVE_WITH_JOYSTICK_FACTOR_MODES = { 1.0, 0.8 };
    public static double DRIVE_WITH_JOYSTICK_FACTOR;

    public static final Gains DRIVE_DISTANCE_GAINS_HIGH = new Gains(0.1, 0.0, 0.0, 0.0, 0, 0.5);
    public static final Gains DRIVE_VELOCITY_GAINS_HIGH = new Gains(0.1, 0.0, 0.0, 0.0, 0, 1.0);
    public static final Gains DRIVE_TURNING_GAINS_HIGH = new Gains(0.2, 0.0, 0.0, 0.0, 0, 0.55);
    public static final Gains DRIVE_MOTION_MAGIC_GAINS_HIGH = new Gains(0.1, 0.0, 0.0, 0.0, 0, 1.0);
    public static final int DRIVE_CRUISE_VELOCITY_HIGH = 20000;
    public static final int DRIVE_ACCELERATION_HIGH = 7000;

    public static final Gains DRIVE_VELOCITY_GAINS_BACK = new Gains(0.0, 0.0, 0.0, 0.05, 0, 1.0);

    /* Trajectory Constants */
    public static final double MAX_SPEED_METERS_PER_SECOND = 1.0;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.0;
    public static final double TRACK_WIDTH_METERS = 0.648;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    /* Remote Sensors */
    public static final int REMOTE_0 = 0;
    public static final int REMOTE_1 = 1;

    /* PID Indexes */
    public static final int PID_PRIMARY = 0;
    public static final int PID_TURN = 1;

    /* PID SLOTS */
    public static final int SLOT_DISTANCE = 0;
    public static final int SLOT_VELOCITY = 1;
    public static final int SLOT_TURNING = 2;
    public static final int SLOT_MOTION_MAGIC = 3;

    /* Ratio Calculation */
    public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double TICK_TIME_TO_SECONDS = 0.1;
    public static final double SECONDS_TO_TICK_TIME = 1 / TICK_TIME_TO_SECONDS;
    public static final double INCHES_PER_METER = 39.370;
    public static final double METERS_PER_INCH = 1 / INCHES_PER_METER;

    public static final double WHEEL_ROT_PER_MOTOR_ROT_HIGH = 1 / MOTOR_ROT_PER_WHEEL_ROT_HIGH;
    public static final double TICKS_PER_WHEEL_REV_HIGH = TICKS_PER_MOTOR_REV * MOTOR_ROT_PER_WHEEL_ROT_HIGH;
    public static final double TICKS_PER_INCH_HIGH = TICKS_PER_WHEEL_REV_HIGH / INCHES_PER_WHEEL_REV;
    public static final double INCHES_PER_TICK_HIGH = 1 / TICKS_PER_INCH_HIGH;

    public static final double WHEEL_ROT_PER_MOTOR_ROT_LOW = 1 / MOTOR_ROT_PER_WHEEL_ROT_LOW;
    public static final double TICKS_PER_WHEEL_REV_LOW = TICKS_PER_MOTOR_REV * MOTOR_ROT_PER_WHEEL_ROT_LOW;
    public static final double TICKS_PER_INCH_LOW = TICKS_PER_WHEEL_REV_LOW / INCHES_PER_WHEEL_REV;
    public static final double INCHES_PER_TICK_LOW = 1 / TICKS_PER_INCH_LOW;
  }

  public static final class ShooterConstants {
    /* Motor IDs */
    public static final int SHOOTER_FALCON_BALLER_ID = 8;
    public static final int SHOOTER_FALCON_BALLER_FOLLOWER_ID = 15;
    public static final int SHOOTER_ANGLE_ADJUST_ID = 10;
    public static final int SHOOTER_ROTATE_ID = 9;

    /* PID Constants Shooter */
    public static final int SHOOTER_SLOT_IDX = 0;
    public static final int SHOOTER_PID_LOOP_IDX = 1;
    public static final int SHOOTER_TIMEOUT_MS = 30;
    public static final Gains DRUM_SHOOTER_GAINS = new Gains(0.34, 0.0, 0.0, 0.055, 0, 1.0); // Ff was 0.055
    // public static final Gains DRUM_SHOOTER_GAINS = new Gains(0.55, 0.0, 100, 0.0, 0, 1.0);
    public static final Gains SHOOTER_TURRET_GAINS = new Gains(0.6, 0.0, 0.0, 0.0, 0, 1.0);
    public static final Gains SHOOTER_ANGLE_GAINS = new Gains(0.05, 0.0, 0.0, 0.0, 0, 0.3);
    public static final double SHOOTER_TURRET_MIN = -1.0;
    public static final double ENCODER_TICKS_PER_REV = 2048;
    public static final double NEO_UNITS_PER_REV = 42;
    public static final double DEGREES_PER_ROT = 360;

    public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(true, 60, 40, 0.5);

    public static final int TURRET_RIGHT_SOFT_LIMIT = -2;
    public static final int TURRET_LEFT_SOFT_LIMIT = -55;
    public static final double TURRET_SPEED_MULTIPLIER = 0.3;
    public static final double TURRET_CALIBRATE_SPEED = 0.075;
    public static final double TURRET_MOTOR_ROTS_PER_ROT = 101.04972; // 89.56696;
    public static final double TURRET_MOTOR_POS_AT_ZERO_ROT = -28.452166;

    public static final int HOOD_UP_SOFT_LIMIT = 33;
    public static final int HOOD_DOWN_SOFT_LIMIT = 3;
    public static final double HOOD_CONVERT_SLOPE = 0.47;
    public static final double HOOD_CONVERT_B = 40.5;
    public static final double HOOD_CALIBRATE_SPEED = 0.2;
    public static final double HOOD_MOTOR_ROTS_PER_ROT = 1; // TODO: Find
    public static final double HOOD_MOTOR_POS_AT_ZERO_ROT = 0; // TODO: Find

    public static final double DRUM_RAMP_LIMIT = 1000;
    public static final double DRUM_VELOCITY_BOUND = 300;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_SPARK_ID = 14;
  }

  public static final class LevelerConstants {
    public static final int LEVELER_CAN_ID = 30;
  }

  public static final class IntakeConstants {
    public static final double EXTENDER_SPEED = 0.3;
    private static final double[] INTAKE_SPEED_MODES = { 1.0, 0.5 };
    public static double INTAKE_SPEED;

    public static final int INTAKE_SPARK_ID = 12;
    public static final int EXTENDER_SPARK_ID = 13;
  }

  public static final class StorageConstants {
    public static final int STORAGE_CAN_ID = 11;
    public static final double STORAGE_PARTIAL_BALL = 2;
    public static final double STORAGE_FULL_BALL = 7;
    private static final double[] STORAGE_SPEED_MODES = { 0.75, 0.50 };
    public static double STORAGE_SPEED;
    public static final double STORAGE_TIMEOUT = 3000;

    /* Storage Characteristics */
    public static final double MOTOR_ROTS_PER_STORAGE_ROT = 1; // For the first storage belt
    public static final double INCHES_PER_STORAGE_ROT = 1; // Circumference of the first storage belt

    /* Ball Indexes */
    public static final int BEAM_SENSOR_SHOOTER = 11;
    public static final int BEAM_SENSOR_USELESS = 12;
    public static final int BEAM_SENSOR_STORAGE = 13;
    public static final int BEAM_SENSOR_INTAKE = 14;

    /* PID Gains */
    public static final Gains STORAGE_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0, 1.0);

    /* PID Values */
    public static final int SLOT_DISTANCE = 0;

    /* PID Indexes */
    public static final int PID_PRIMARY = 0;
  }

  public static final class PneumaticsConstants {
    public static final int PCM_MODULE_ID = 7;

    public static final int SPEED_SHIFT_FORWARD_ID = 0;
    public static final int SPEED_SHIFT_REVERSE_ID = 1;

    public static final int COOL_FALCON_FORWARD_ID = 3;
    public static final int COOL_FALCON_REVERSE_ID = 2;
  }

  public static final class LEDConstants {
    public static final int LED_SPARK_ID = 0;
    public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
  }

  public static final class VisionConstants {
    public static final double FOV = 29.8; // Field of view of limelight
    public static final double TARGET_HEIGHT = 67.5;
    public static final double LIME_ANGLE = 24.7;
    public static final double TURN_P_VALUE = 0.8;
    public static final double X_ANGLE_ERROR = 0.5;
    public static final double MOTOR_DEAD_ZONE = 0.2;
    public static final double DISTANCE_ERROR_EQUATION_M = 1.1279;
    public static final double DISTANCE_ERROR_EQUATION_B = -15.0684;
    public static final double GRAV = 385.83;

    // Galactic Search
    public static final double searchError = 2;
    /*
    public static final double bothCloseVisibleY = -17.69;
    public static final double closeLeftVisibleY = -12.57;
    public static final double closeRightVisibleY = -11.35;
    public static final double farLeftVisibleX = 3.58;
    public static final double farRightVisibleX = 7.04;
    */

    public static final double[] aRed = { 1.6, -11.7 };
    public static final double[] bRed = { 2.5, -5.5 };
    public static final double[] aBlue = { 9.9, 9.0 };
    public static final double[] bBlue = { 5.5, 13.3 };
  }

  public static final class OIConstants {
    public static final int XBOX_DRIVER_ID = 0;
    public static final int XBOX_OPERATOR_ID = 1;
    public static final int BUTTON_FOX_ID = 2;
  }
}
