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
import com.ctre.phoenix.motorcontrol.SensorTerm;
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
    m_pigeon.configFactoryDefault();
    resetGyroYaw();

    /* set back motors as followers */
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    /* set neutral mode */
    setDriveTrainNeutralMode(NeutralMode.Brake);  
    
    /* flip input so forward becomes back, etc */
    m_leftFrontMotor.setInverted(false);
    m_rightFrontMotor.setInverted(false);
    m_leftBackMotor.setInverted(InvertType.FollowMaster);
    m_rightBackMotor.setInverted(InvertType.FollowMaster);

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

    SmartDashboard.putNumber("P Value Drive Distance", DriveConstants.DRIVE_DISTANCE_GAINS.kP);
    SmartDashboard.putNumber("I Value Drive Distance", DriveConstants.DRIVE_DISTANCE_GAINS.kI);
    SmartDashboard.putNumber("D Value Drive Distance", DriveConstants.DRIVE_DISTANCE_GAINS.kD);
    SmartDashboard.putNumber("F Value Drive Distance", DriveConstants.DRIVE_DISTANCE_GAINS.kF);

    SmartDashboard.putNumber("P Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kP);
    SmartDashboard.putNumber("I Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kI);
    SmartDashboard.putNumber("D Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kD);
    SmartDashboard.putNumber("F Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kF);

    SmartDashboard.putNumber("P Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kP);
    SmartDashboard.putNumber("I Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kI);
    SmartDashboard.putNumber("D Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kD);
    SmartDashboard.putNumber("F Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kF);

    SmartDashboard.putNumber("P Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kP);
    SmartDashboard.putNumber("I Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kI);
    SmartDashboard.putNumber("D Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kD);
    SmartDashboard.putNumber("F Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kF);

    int closedLoopTimeMs = 1;
    m_leftFrontMotor.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.DRIVE_TIMEOUT_MS);
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

      m_gainsDistance.kP = SmartDashboard.getNumber("P Value Drive", DriveConstants.DRIVE_DISTANCE_GAINS.kP);
      m_gainsDistance.kI = SmartDashboard.getNumber("I Value Drive", DriveConstants.DRIVE_DISTANCE_GAINS.kI);
      m_gainsDistance.kD = SmartDashboard.getNumber("D Value Drive", DriveConstants.DRIVE_DISTANCE_GAINS.kD);
      m_gainsDistance.kF = SmartDashboard.getNumber("F Value Drive", DriveConstants.DRIVE_DISTANCE_GAINS.kF);

      m_gainsVelocity.kP = SmartDashboard.getNumber("P Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kP);
      m_gainsVelocity.kI = SmartDashboard.getNumber("I Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kI);
      m_gainsVelocity.kD = SmartDashboard.getNumber("D Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kD);
      m_gainsVelocity.kF = SmartDashboard.getNumber("F Value Drive Velocity", DriveConstants.DRIVE_VELOCITY_GAINS.kF);

      m_gainsTurning.kP = SmartDashboard.getNumber("P Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kP);
      m_gainsTurning.kI = SmartDashboard.getNumber("I Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kI);
      m_gainsTurning.kD = SmartDashboard.getNumber("D Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kD);
      m_gainsTurning.kF = SmartDashboard.getNumber("F Value Drive Turning", DriveConstants.DRIVE_TURNING_GAINS.kF);

      m_gainsMotionMagic.kP = SmartDashboard.getNumber("P Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kP);
      m_gainsMotionMagic.kI = SmartDashboard.getNumber("I Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kI);
      m_gainsMotionMagic.kD = SmartDashboard.getNumber("D Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kD);
      m_gainsMotionMagic.kF = SmartDashboard.getNumber("F Value Drive Motion Magic", DriveConstants.DRIVE_MOTION_MAGIC_GAINS.kF);

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
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_DISTANCE, m_gainsDistance.kD, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Velocity */
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_VELOCITY, m_gainsVelocity.kD, DriveConstants.DRIVE_TIMEOUT_MS);
    
    /* Turning */
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_TURNING, m_gainsTurning.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_TURNING, m_gainsTurning.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_TURNING, m_gainsTurning.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_TURNING, m_gainsTurning.kD, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Motion Magic */
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_MOTION_MAGIC, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.config_kF(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagic.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagic.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagic.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagic.kD, DriveConstants.DRIVE_TIMEOUT_MS);

    m_rightFrontMotor.configMotionCruiseVelocity(DriveConstants.DRIVE_CRUISE_VELOCITY, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configMotionAcceleration(DriveConstants.DRIVE_ACCELERATION, DriveConstants.DRIVE_TIMEOUT_MS);
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
