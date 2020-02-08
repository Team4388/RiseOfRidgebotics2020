/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.Gains;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonFX m_leftFrontMotor = new WPI_TalonFX(DriveConstants.DRIVE_LEFT_FRONT_CAN_ID);
  public WPI_TalonFX m_rightFrontMotor = new WPI_TalonFX(DriveConstants.DRIVE_RIGHT_FRONT_CAN_ID);
  public WPI_TalonFX m_leftBackMotor = new WPI_TalonFX(DriveConstants.DRIVE_LEFT_BACK_CAN_ID);
  public WPI_TalonFX m_rightBackMotor = new WPI_TalonFX(DriveConstants.DRIVE_RIGHT_BACK_CAN_ID);
  public static PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.PIGEON_ID);

  public DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

  SendableChooser<Gains> m_chooser = new SendableChooser<Gains>();
  public static Gains m_gainsDistance = DriveConstants.DRIVE_DISTANCE_GAINS;
  public static Gains m_gainsVelocity = DriveConstants.DRIVE_VELOCITY_GAINS;
  public static Gains m_gainsTurning = DriveConstants.DRIVE_TURNING_GAINS;
  public static Gains m_gainsMotionMagic = DriveConstants.DRIVE_MOTION_MAGIC_GAINS;

  /**
   * Add your docs here.
   */
  public Drive() {
    /* factory default values */
    m_leftFrontMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_leftBackMotor.configFactoryDefault();
    m_rightBackMotor.configFactoryDefault();
    //m_pigeon.configFactoryDefault();
    //resetGyroYaw();

    /* set back motors as followers */
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    setDriveTrainNeutralMode(NeutralMode.Coast);

    /* deadbands */
    m_leftBackMotor.configNeutralDeadband(0.0, DriveConstants.DRIVE_TIMEOUT_MS); // DO NOT CHANGE
    m_rightBackMotor.configNeutralDeadband(0.0, DriveConstants.DRIVE_TIMEOUT_MS); //Ensures motors run at the same speed

    /* flip input so forward becomes back, etc */
    m_leftFrontMotor.setInverted(false);
    m_rightFrontMotor.setInverted(true);
    m_driveTrain.setRightSideInverted(false);
    m_leftBackMotor.setInverted(InvertType.FollowMaster);
    m_rightBackMotor.setInverted(InvertType.FollowMaster);


    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kD, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);

    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_TURNING, m_gainsTurning.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_TURNING, m_gainsTurning.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_TURNING, m_gainsTurning.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_TURNING, m_gainsTurning.kD, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_TURNING, m_gainsTurning.kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);
    
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kD, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);    

    /* Setup Sensors for WPI_TalonFXs */
    m_leftFrontMotor.setSelectedSensorPosition(0, DriveConstants.PID_PRIMARY, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.setSelectedSensorPosition(0, DriveConstants.PID_PRIMARY, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Configure the left Talon's selected sensor as local QuadEncoder */
		m_leftFrontMotor.configSelectedFeedbackSensor(  FeedbackDevice.IntegratedSensor,    // Local Feedback Source
                                                    DriveConstants.PID_PRIMARY,				  // PID Index for Source [0, 1]
                                                    DriveConstants.DRIVE_TIMEOUT_MS);	  // Configuration Timeout
                                                    
    /*m_rightFrontMotor.configSelectedFeedbackSensor(  FeedbackDevice.IntegratedSensor, // Local Feedback Source
                                                    DriveConstants.PID_PRIMARY,				  // PID Index for Source [0, 1]
                                                    DriveConstants.DRIVE_TIMEOUT_MS);*/	// Configuration Timeout

    /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		m_rightFrontMotor.configRemoteFeedbackFilter( m_leftFrontMotor.getDeviceID(),					      // Device ID of Source
                                                  RemoteSensorSource.TalonSRX_SelectedSensor,
                                                  DriveConstants.REMOTE_0,							        // Source number [0, 1]
                                                  DriveConstants.DRIVE_TIMEOUT_MS);				  // Configuration Timeout

    /* Configure the Pigeon IMU to the other Remote Slot available on the right Talon */
		m_rightFrontMotor.configRemoteFeedbackFilter( m_pigeon.getDeviceID(),
                                                  RemoteSensorSource.Pigeon_Yaw,
                                                  DriveConstants.REMOTE_1,	
                                                  DriveConstants.DRIVE_TIMEOUT_MS);

    /* Setup Sum signal to be used for Distance */
    m_rightFrontMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.IntegratedSensor, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Diff Signal */
    m_rightFrontMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.IntegratedSensor, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		m_rightFrontMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference,
                                                    DriveConstants.PID_PRIMARY,
                                                    DriveConstants.DRIVE_TIMEOUT_MS);
 
    /* Scale Feedback by 0.5 to half the sum of Distance */
	  m_rightFrontMotor.configSelectedFeedbackCoefficient(  1, 						                    // Coefficient
                                                          DriveConstants.PID_PRIMARY,		      // PID Slot of Source 
                                                          DriveConstants.DRIVE_TIMEOUT_MS);   // Configuration Timeout

    m_rightFrontMotor.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1, 
                                                    DriveConstants.PID_TURN,
                                                    DriveConstants.DRIVE_TIMEOUT_MS);

    /* Don't scale the Feedback Sensor (use 1 for 1:1 ratio) */
    m_rightFrontMotor.configSelectedFeedbackCoefficient(  1,
                                                          DriveConstants.PID_TURN,
                                                          DriveConstants.DRIVE_TIMEOUT_MS);

    /* Don't scale the Feedback Sensor (use 1 for 1:1 ratio) */
	  m_leftFrontMotor.configSelectedFeedbackCoefficient( 1,
                                                        DriveConstants.PID_PRIMARY,
                                                        DriveConstants.DRIVE_TIMEOUT_MS);
                                                        
   /* Set status frame periods to ensure we don't have stale data */
		m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, DriveConstants.DRIVE_TIMEOUT_MS);
		m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.DRIVE_TIMEOUT_MS);
		m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, DriveConstants.DRIVE_TIMEOUT_MS);
    m_pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Smart Dashboard Initial Values */

      /* Set up Chooser */
    m_chooser.setDefaultOption("Distance PID", m_gainsDistance);
    //setDriveTrainGains("Distance PID", m_gainsDistance);
    m_chooser.addOption("Velocity PID", m_gainsVelocity);
    //setDriveTrainGains("Velocity PID", m_gainsVelocity);
    m_chooser.addOption("Turning PID", m_gainsTurning);
    //setDriveTrainGains("Turning PID", m_gainsTurning);
    m_chooser.addOption("Motion Magic PID", m_gainsMotionMagic);
    //setDriveTrainGains("Motion Magic PID", m_gainsMotionMagic);
    Shuffleboard.getTab("PID").add(m_chooser);

      /* Gyro */
    SmartDashboard.putNumber("Pigeon Yaw", getGyroYaw());
    SmartDashboard.putNumber("Pigeon Pitch", getGyroPitch());
    SmartDashboard.putNumber("Pigeon Roll", getGyroRoll());

      /* Sensor Values */
    SmartDashboard.putNumber("Left Motor Velocity Raw", m_leftFrontMotor.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Right Motor Velocity Raw", m_rightFrontMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Motor Position Raw", m_leftFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Motor Position Raw", m_rightFrontMotor.getSelectedSensorPosition());

      /* PID */
    Gains gains = m_chooser.getSelected();
    Shuffleboard.getTab("PID").add("P Value Drive", gains.kP);
    Shuffleboard.getTab("PID").add("I Value Drive", gains.kI);
    Shuffleboard.getTab("PID").add("D Value Drive", gains.kD);
    Shuffleboard.getTab("PID").add("F Value Drive", gains.kF);

        
   /**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_leftFrontMotor.configPeakOutputForward(+1, DriveConstants.DRIVE_TIMEOUT_MS);
		m_leftFrontMotor.configPeakOutputReverse(-1, DriveConstants.DRIVE_TIMEOUT_MS);
		m_rightFrontMotor.configPeakOutputForward(+1, DriveConstants.DRIVE_TIMEOUT_MS);
		m_rightFrontMotor.configPeakOutputReverse(-1, DriveConstants.DRIVE_TIMEOUT_MS);

    /**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
    int closedLoopTimeMs = 1;
    m_rightFrontMotor.configClosedLoopPeriod(DriveConstants.PID_PRIMARY, closedLoopTimeMs, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configClosedLoopPeriod(DriveConstants.PID_TURN, closedLoopTimeMs, DriveConstants.DRIVE_TIMEOUT_MS);
    /**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
    m_rightFrontMotor.configAuxPIDPolarity(false, DriveConstants.DRIVE_TIMEOUT_MS);
  }

  @Override
  public void periodic() {
    try {
      SmartDashboard.putNumber("Pigeon Yaw", getGyroYaw());
      SmartDashboard.putNumber("Pigeon Pitch", getGyroPitch());
      SmartDashboard.putNumber("Pigeon Roll", getGyroRoll());

      SmartDashboard.putNumber("Left Motor Velocity Raw", m_leftFrontMotor.getSelectedSensorVelocity(0));
      SmartDashboard.putNumber("Right Motor Velocity Raw", m_rightFrontMotor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Left Motor Position Raw", m_leftFrontMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Right Motor Position Raw", m_rightFrontMotor.getSelectedSensorPosition(0));
      SmartDashboard.putNumber("Right Motor Velocity Int Sensor", m_rightFrontMotor.getSensorCollection().getIntegratedSensorVelocity());
      SmartDashboard.putNumber("Left Motor Velocity Int Sensor", m_leftFrontMotor.getSensorCollection().getIntegratedSensorVelocity());

      SmartDashboard.putNumber("Right Front Motor Current", m_rightFrontMotor.getSupplyCurrent());
      SmartDashboard.putNumber("Left Front Motor Current", m_leftFrontMotor.getSupplyCurrent());
      SmartDashboard.putNumber("Right Back Motor Current", m_rightFrontMotor.getSupplyCurrent());
      SmartDashboard.putNumber("Left Back Motor Current", m_leftFrontMotor.getSupplyCurrent());

      SmartDashboard.putNumber("PID 0 Error", m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_PRIMARY));
      SmartDashboard.putNumber("PID 1 Error", m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_TURN));
      SmartDashboard.putNumber("PID 0 Target", m_rightFrontMotor.getClosedLoopTarget(DriveConstants.PID_PRIMARY));
      SmartDashboard.putNumber("PID 1 Target", m_rightFrontMotor.getClosedLoopTarget(DriveConstants.PID_TURN));
      SmartDashboard.putNumber("PID 0 Pos", m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_PRIMARY));
      SmartDashboard.putNumber("PID 1 Pos", m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN));

    } catch (Exception e) {
      System.err.println("Error in the Drive Subsystem");
      //e.printStackTrace(System.err);
    }
  }

  /**
   * Sets Motors to a NeutralMode.
   * @param mode NeutralMode to set motors to
   */
  public void setDriveTrainNeutralMode(NeutralMode mode) {
    m_leftFrontMotor.setNeutralMode(mode);
    m_rightFrontMotor.setNeutralMode(mode);
    m_leftBackMotor.setNeutralMode(mode);
    m_rightBackMotor.setNeutralMode(mode);
  }

  /**
   * Initializes the drive train gains kP, kI, kD, and kF
   * @param slot Either "Distance PID", "Velocity PID", "Motion Magic PID", or "Turning PID"
   * @param gains A gains object which is the gains that are set for the slot
   */
  public void setDriveTrainGains(String slot, Gains gains){
    /* Distance */
    if (slot.equals("Distance PID")) {
      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_DISTANCE, gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_DISTANCE, gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_DISTANCE, gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_DISTANCE, gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);
    }

    /* Velocity */
    if (slot.equals("Velocity PID")) {
      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_VELOCITY, gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_VELOCITY, gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_VELOCITY, gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_VELOCITY, gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_VELOCITY, gains.kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);
    }
    /* Turning */
    if (slot.equals("Turning PID")) {
      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_TURNING, gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_TURNING, gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_TURNING, gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_TURNING, gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_TURNING, gains.kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);
    }

    /* Motion Magic */
    if (slot.equals("Motion Magic PID")) {
      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_MOTION_MAGIC, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_MOTION_MAGIC, gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_MOTION_MAGIC, gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_MOTION_MAGIC, gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_MOTION_MAGIC, gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);
    
      m_rightFrontMotor.configMotionCruiseVelocity(DriveConstants.DRIVE_CRUISE_VELOCITY, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionAcceleration(DriveConstants.DRIVE_ACCELERATION, DriveConstants.DRIVE_TIMEOUT_MS);
    }
  }

  /**
   * Add your docs here.
   */
  public void driveWithInput(double move, double steer){
    m_driveTrain.arcadeDrive(move, steer);
  }
  /**
   * Runs a position PID while driving straight (has not been tested)
   * @param targetPos The position to drive to in units
   * @param targetGyro The angle to drive at in units
   */
  public void runDriveStraightPositionPID(double targetPos, double targetGyro) {
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);

    targetPos *= 2;
    m_rightFrontMotor.set(TalonFXControlMode.Position, targetPos, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);

    m_driveTrain.feedWatchdog();
  }
  /**
   * Runs velocity PID while driving straight
   * @param targetVel The velocity to drive at in units
   * @param targetGyro The angle to drive at in units
   */
  public void runDriveStraightVelocityPID(double targetVel, double targetGyro) {
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);

    targetVel *= 2;
    m_rightFrontMotor.set(TalonFXControlMode.Velocity, targetVel, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);

    m_driveTrain.feedWatchdog();
  }
  /**
   * Runs motion magic PID while driving straight (has not been tested)
   * @param targetPos The position to drive to in units
   * @param targetGyro The angle to drive at in units
   */
  public void runMotionMagicPID(double targetPos, double targetGyro){
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_MOTION_MAGIC, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);
    m_rightFrontMotor.set(TalonFXControlMode.MotionMagic, targetPos, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);
    m_driveTrain.feedWatchdog();
  }

  /**
   * Runs a Turning PID to rotate a to a target angle
   * @param targetAngle target angle in degrees
   */
  public void runTurningPID(double targetAngle){
    double targetGyro = (targetAngle/360)*DriveConstants.TICKS_PER_GYRO_REV;
    runDriveStraightVelocityPID(0, targetGyro);
  }

  public double getGyroYaw() {
    double[] ypr = new double[3];
    m_pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getGyroPitch() {
    double[] ypr = new double[3];
    m_pigeon.getYawPitchRoll(ypr);
    return ypr[1];
  }

  public double getGyroRoll() {
    double[] ypr = new double[3];
    m_pigeon.getYawPitchRoll(ypr);
    return ypr[2];
  }

  public void resetGyroYaw() {
    m_pigeon.setYaw(0);
    m_pigeon.setAccumZAngle(0);
  }
}
