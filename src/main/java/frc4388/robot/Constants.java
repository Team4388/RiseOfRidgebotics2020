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
        public static final int DRIVE_LEFT_FRONT_CAN_ID = 2;
        public static final int DRIVE_RIGHT_FRONT_CAN_ID = 4;
	    public static final int DRIVE_LEFT_BACK_CAN_ID = 3;
        public static final int DRIVE_RIGHT_BACK_CAN_ID = 5;
        public static final int PIGEON_ID = 6;

        /* PID Constants Drive*/
        public static final int DRIVE_SLOT_IDX = 0;
        public static final int DRIVE_PID_LOOP_IDX = 0;
        public static final int DRIVE_TIMEOUT_MS = 30;
        public static final Gains DRIVE_GAINS = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
        
        public static final double ENCODER_TICKS_PER_REV = 2048;
    }
    public static final class ElevatorConstants{
        public static final int ELEVATOR_CAN_ID = 0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_SPARK_ID = 1;
    }

    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }
    public static final class VisionConstants {
    public static final double FOV = 29.8; //Field of view of limelight
    public static final double TARGET_HEIGHT = 82.75;
    public static final double LIME_ANGLE = 24.11;
    public static final double TURN_P_VALUE = 0.65;
    public static final double X_ANGLE_ERROR = 1.3;
    public static final double MOTOR_DEAD_ZONE = 0.3;
    public static final double DISTANCE_ERROR_EQUATION_M = 1.1279;
    public static final double DISTANCE_ERROR_EQUATION_B = -15.0684;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
    }
}
