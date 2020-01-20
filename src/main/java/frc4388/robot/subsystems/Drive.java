/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  public static Gains m_gains = DriveConstants.DRIVE_GAINS;

  /**
   * Add your docs here.
   */
  public Drive() {
    /* factory default values */
    m_leftFrontMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_leftBackMotor.configFactoryDefault();
    m_rightBackMotor.configFactoryDefault();
    m_pigeon.configFactoryDefault();
    resetGyroYaw();

    /* set back motors as followers */
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    /* set neutral mode */
    setDriveTrainNeutralMode(NeutralMode.Brake);

    /* Setup Sensors for TalonFXs */
    
    /* Configure the left Talon's selected sensor as local QuadEncoder */
		m_leftFrontMotor.configSelectedFeedbackSensor(  FeedbackDevice.IntegratedSensor,    // Local Feedback Source
                                                    DriveConstants.PID_PRIMARY,				  // PID Index for Source [0, 1]
                                                    DriveConstants.DRIVE_TIMEOUT_MS);	  // Configuration Timeout

    /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		m_rightFrontMotor.configRemoteFeedbackFilter( m_leftFrontMotor.getDeviceID(),					      // Device ID of Source
                                                  RemoteSensorSource.TalonSRX_SelectedSensor,   // Remote Feedback Source
                                                  DriveConstants.REMOTE_0,							        // Source number [0, 1]
                                                  DriveConstants.DRIVE_TIMEOUT_MS);						  // Configuration Timeout

    /* Setup Sum signal to be used for Distance */
    m_rightFrontMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.IntegratedSensor, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		m_rightFrontMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
                                                    DriveConstants.PID_PRIMARY,
                                                    DriveConstants.DRIVE_TIMEOUT_MS);

    /* Scale Feedback by 0.5 to half the sum of Distance */
	  m_rightFrontMotor.configSelectedFeedbackCoefficient(  0.5, 						                    // Coefficient
                                                          DriveConstants.PID_PRIMARY,		      // PID Slot of Source 
                                                          DriveConstants.DRIVE_TIMEOUT_MS);   // Configuration Timeout

    /* Scale the Feedback Sensor using a coefficient */
	  m_rightFrontMotor.configSelectedFeedbackCoefficient(	1.0,
                                                          DriveConstants.PID_TURN,
                                                          DriveConstants.DRIVE_TIMEOUT_MS);
    
    /* flip input so forward becomes back, etc */
    m_leftFrontMotor.setInverted(false);
    m_rightFrontMotor.setInverted(false);
    m_leftBackMotor.setInverted(InvertType.FollowMaster);
    m_rightBackMotor.setInverted(InvertType.FollowMaster);
    m_rightFrontMotor.setSensorPhase(false);
    m_leftFrontMotor.setSensorPhase(false);

    /* Set status frame periods to ensure we don't have stale data */
		m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, DriveConstants.DRIVE_TIMEOUT_MS);
		m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.DRIVE_TIMEOUT_MS);
		m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, DriveConstants.DRIVE_TIMEOUT_MS);
		m_leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, DriveConstants.DRIVE_TIMEOUT_MS);

    /* deadbands */
    m_leftBackMotor.configNeutralDeadband(0.0); // DO NOT CHANGE
    m_rightBackMotor.configNeutralDeadband(0.0); //Ensures motors run at the same speed

    setDriveTrainGains();   
    m_leftFrontMotor.setSelectedSensorPosition(0, DriveConstants.PID_PRIMARY, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.setSelectedSensorPosition(0, DriveConstants.PID_PRIMARY, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Smart Dashboard Initial Values */
    SmartDashboard.putNumber("Pigeon Yaw", getGyroYaw());
    SmartDashboard.putNumber("Pigeon Pitch", getGyroPitch());
    SmartDashboard.putNumber("Pigeon Roll", getGyroRoll());

    SmartDashboard.putNumber("Left Motor Velocity Raw", m_leftFrontMotor.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Right Motor Velocity Raw", m_rightFrontMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Motor Position Raw", m_leftFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Motor Position Raw", m_rightFrontMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("P Value Drive", DriveConstants.DRIVE_GAINS.kP);
    SmartDashboard.putNumber("I Value Drive", DriveConstants.DRIVE_GAINS.kI);
    SmartDashboard.putNumber("D Value Drive", DriveConstants.DRIVE_GAINS.kD);
    SmartDashboard.putNumber("F Value Drive", DriveConstants.DRIVE_GAINS.kF);


    /**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
    int closedLoopTimeMs = 1;
    m_leftFrontMotor.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.DRIVE_TIMEOUT_MS);

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

      m_gains.kP = SmartDashboard.getNumber("P Value Drive", DriveConstants.DRIVE_GAINS.kP);
      m_gains.kI = SmartDashboard.getNumber("I Value Drive", DriveConstants.DRIVE_GAINS.kI);
      m_gains.kD = SmartDashboard.getNumber("D Value Drive", DriveConstants.DRIVE_GAINS.kD);
      m_gains.kF = SmartDashboard.getNumber("F Value Drive", DriveConstants.DRIVE_GAINS.kF);

      setDriveTrainGains();

    } catch (Exception e) {

      System.err.println("The programming team has failed successfully in the Drive Subsystem.");
    
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
   * Add your docs here.
   */
  public void setDriveTrainGains(){
    /* Distance */
    m_leftFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
    m_leftFrontMotor.config_kF(DriveConstants.SLOT_DISTANCE, m_gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.config_kP(DriveConstants.SLOT_DISTANCE, m_gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.config_kI(DriveConstants.SLOT_DISTANCE, m_gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.config_kD(DriveConstants.SLOT_DISTANCE, m_gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Motion Magic */
    m_leftFrontMotor.configMotionCruiseVelocity(DriveConstants.DRIVE_CRUISE_VELOCITY, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configMotionAcceleration(DriveConstants.DRIVE_ACCELERATION, DriveConstants.DRIVE_TIMEOUT_MS);
  }

  /**
   * Add your docs here.
   */
  public void driveWithInput(double move, double steer){
    m_driveTrain.arcadeDrive(move, steer);
  }

  public void runPositionPID(WPI_TalonFX talon, double targetPos) {
    talon.set(TalonFXControlMode.Position, targetPos);
  }

  public void runVelocityPID(WPI_TalonFX talon, double targetVel) {
    talon.set(TalonFXControlMode.Velocity, targetVel);
  }

  public void runMotionMagicPID(WPI_TalonFX talon, double targetPos){
    talon.set(TalonFXControlMode.MotionMagic, targetPos);
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
