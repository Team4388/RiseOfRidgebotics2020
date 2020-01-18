/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

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

        /* PID Constants Drive*/
        public static final int DRIVE_TIMEOUT_MS = 30;
        public static final Gains DRIVE_GAINS = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
        public static final int DRIVE_CRUISE_VELOCITY = 15000;
        public static final int DRIVE_ACCELERATION = 6000;

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
        public static final double MOTOR_TO_WHEEL_GEAR_RATIO = 12.5;
        public static final double WHEEL_DIAMETER_INCHES = 6;
        
        /* Ratio Calculation */
        public static final double TICK_TIME_TO_SECONDS = 0.1;
        public static final double SECONDS_TO_TICK_TIME = 1/TICK_TIME_TO_SECONDS;
        public static final double WHEEL_TO_MOTOR_GEAR_RATIO = 1/MOTOR_TO_WHEEL_GEAR_RATIO;
        public static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_TO_WHEEL_GEAR_RATIO;
        public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
        public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REV/INCHES_PER_WHEEL_REV;
        public static final double INCHES_PER_TICK = 1/TICKS_PER_INCH;
    }
    
    public static final class IntakeConstants {
        public static final int INTAKE_SPARK_ID = 1;
    }

    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
    }
}
