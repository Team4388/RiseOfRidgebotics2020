/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc4388.utility.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        /* Drive Train IDs */
        public static final int DRIVE_LEFT_FRONT_CAN_ID = 2;
        public static final int DRIVE_RIGHT_FRONT_CAN_ID = 4;
	    public static final int DRIVE_LEFT_BACK_CAN_ID = 3;
        public static final int DRIVE_RIGHT_BACK_CAN_ID = 5;
        public static final int PIGEON_ID = 6;

        /* Drive Inversions */
        public static final boolean isRightMotorInverted = false;
        public static final boolean isLeftMotorInverted = false;
        public static final boolean isRightArcadeInverted = false;
        public static final boolean isAuxPIDInverted = false;

        /* Drive Configuration */
        public static final double OPEN_LOOP_RAMP_RATE = 0.1;
        public static final double NEUTRAL_DEADBAND = 0.04;
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIG =
            new SupplyCurrentLimitConfiguration(false, 40, 35, 0.01);
        public static final int CLOSED_LOOP_TIME_MS = 1;

        /* PID Constants Drive*/
        public static final int DRIVE_TIMEOUT_MS = 30;
        public static final Gains DRIVE_DISTANCE_GAINS_LOW = new Gains(0.1, 0.0, 1.0, 0.0, 0, 0.5);
        public static final Gains DRIVE_VELOCITY_GAINS_LOW = new Gains(0.1, 0.0, 0.2, 0.025, 0, 1.0);
        public static final Gains DRIVE_TURNING_GAINS_LOW = new Gains(0.5, 0.0, 0.05, 0.0, 0, 0.55);
        public static final Gains DRIVE_MOTION_MAGIC_GAINS_LOW = new Gains(0.2, 0.0, 0.0, 0.0, 0, 1.0);
        public static final int DRIVE_CRUISE_VELOCITY = 20000;
        public static final int DRIVE_ACCELERATION = 7000;

        public static final Gains DRIVE_DISTANCE_GAINS_HIGH = new Gains(0.0, 0.0, 0.0, 0.0, 0, 0.5);
        public static final Gains DRIVE_VELOCITY_GAINS_HIGH = new Gains(0.0, 0.0, 0.0, 0.0, 0, 1.0);
        public static final Gains DRIVE_TURNING_GAINS_HIGH = new Gains(0.0, 0.0, 0.0, 0.0, 0, 0.55);
        public static final Gains DRIVE_MOTION_MAGIC_GAINS_HIGH = new Gains(0.0, 0.0, 0.0, 0.0, 0, 1.0);
        public static final int DRIVE_CRUISE_VELOCITY_HIGH = 20000;
        public static final int DRIVE_ACCELERATION_HIGH = 7000;

        /* Trajectory Constants */
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double TRACK_WIDTH_METERS = 0.648;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
 
        /* Remote Sensors */
        public final static int REMOTE_0 = 0;
        public final static int REMOTE_1 = 1;
        
        /* PID Indexes */
        public final static int PID_PRIMARY = 0;
        public final static int PID_TURN = 1;
        
        /* PID SLOTS */
        public final static int SLOT_DISTANCE = 0;
	    public final static int SLOT_VELOCITY = 1;
	    public final static int SLOT_TURNING = 2;
	    public final static int SLOT_MOTION_MAGIC = 3;
        
        /* Drive Train Characteristics */
        public static final double TICKS_PER_MOTOR_REV = 2048;
        public static final double MOTOR_ROT_PER_WHEEL_ROT_HIGH = 5.13;
        public static final double MOTOR_ROT_PER_WHEEL_ROT_LOW = 15;
        public static final double WHEEL_DIAMETER_INCHES = 6;
        public static final double TICKS_PER_GYRO_REV = 8192;
        
        /* Ratio Calculation */
        public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
        public static final double TICK_TIME_TO_SECONDS = 0.1;
        public static final double SECONDS_TO_TICK_TIME = 1/TICK_TIME_TO_SECONDS;
        public static final double INCHES_PER_METER = 39.370;
        public static final double METERS_PER_INCH = 1/INCHES_PER_METER;
        
        public static final double WHEEL_ROT_PER_MOTOR_ROT_HIGH = 1/MOTOR_ROT_PER_WHEEL_ROT_HIGH;
        public static final double TICKS_PER_WHEEL_REV_HIGH = TICKS_PER_MOTOR_REV * MOTOR_ROT_PER_WHEEL_ROT_HIGH;
        public static final double TICKS_PER_INCH_HIGH = TICKS_PER_WHEEL_REV_HIGH/INCHES_PER_WHEEL_REV;
        public static final double INCHES_PER_TICK_HIGH = 1/TICKS_PER_INCH_HIGH;

        public static final double WHEEL_ROT_PER_MOTOR_ROT_LOW = 1/MOTOR_ROT_PER_WHEEL_ROT_LOW;
        public static final double TICKS_PER_WHEEL_REV_LOW = TICKS_PER_MOTOR_REV * MOTOR_ROT_PER_WHEEL_ROT_LOW;
        public static final double TICKS_PER_INCH_LOW = TICKS_PER_WHEEL_REV_LOW/INCHES_PER_WHEEL_REV;
        public static final double INCHES_PER_TICK_LOW = 1/TICKS_PER_INCH_LOW;
    }
    
    public static final class IntakeConstants {
        public static final int INTAKE_SPARK_ID = -9;
        public static final int EXTENDER_SPARK_ID = -10;
    }
  
    public static final class ShooterConstants {
        /* Motor IDs */
        public static final int SHOOTER_FALCON_ID = -1;
        public static final int SHOOTER_ANGLE_ADJUST_ID = -1;
        public static final int SHOOTER_ROTATE_ID = 10;

        /* PID Constants Shooter */
        public static final int SHOOTER_SLOT_IDX = 0;
        public static final int SHOOTER_PID_LOOP_IDX = 1;
        public static final int SHOOTER_TIMEOUT_MS = 30;
        public static final Gains DRUM_SHOOTER_GAINS = new Gains(0.4, 0.0005, 13, 0.05, 0, 1.0);
        public static final Gains SHOOTER_TURRET_GAINS = new Gains(0.2, 0.0, 0.0, 0.0, 0, 1.0);
        public static final double SHOOTER_TURRET_MIN = -1.0;
        public static final double ENCODER_TICKS_PER_REV = 2048;
        public static final double NEO_UNITS_PER_REV = 42;
        public static final double DEGREES_PER_ROT = 360;
    }
    
    public static final class ClimberConstants {
        public static final int CLIMBER_SPARK_ID = 10;
    }
  
    public static final class LevelerConstants {
        public static final int LEVELER_CAN_ID = -1;
    }
  
    public static final class StorageConstants {
        public static final int STORAGE_CAN_ID = -1;

        /* Ball Indexes */
        public static final int BEAM_SENSOR_DIO_0 = 0;
        public static final int BEAM_SENSOR_DIO_1 = 1;
        public static final int BEAM_SENSOR_DIO_2 = 2;
        public static final int BEAM_SENSOR_DIO_3 = 3;
        public static final int BEAM_SENSOR_DIO_4 = 4;
        public static final int BEAM_SENSOR_DIO_5 = 5;

        /* PID Values */
        public static final int SLOT_DISTANCE = 0;

        /* PID Indexes */
        public static final int PID_PRIMARY = 0;

        /* PID Gains */

        public static final double STORAGE_MIN_OUTPUT = -1.0;
        public static final Gains STORAGE_GAINS = new Gains(0.2, 0.0, 0.0, 0.0, 0, 1.0);
    }
  
    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;
        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }
    
    public static final class VisionConstants {
        public static final double FOV = 29.8; //Field of view of limelight
        public static final double TARGET_HEIGHT = 82.75;
        public static final double LIME_ANGLE = 18.7366;
        public static final double TURN_P_VALUE = 0.65;
        public static final double X_ANGLE_ERROR = 1.3;
        public static final double MOTOR_DEAD_ZONE = 0.3;
        public static final double DISTANCE_ERROR_EQUATION_M = 1.1279;
        public static final double DISTANCE_ERROR_EQUATION_B = -15.0684;
        public static final double GRAV = 385.83;
        }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
    }
}
