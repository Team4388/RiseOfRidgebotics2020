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

    /* flip input so forward becomes back, etc */
    m_leftFrontMotor.setInverted(false);
    m_rightFrontMotor.setInverted(false);
    m_leftBackMotor.setInverted(InvertType.FollowMaster);
    m_rightBackMotor.setInverted(InvertType.FollowMaster);

    m_leftBackMotor.configNeutralDeadband(0.0); // DO NOT CHANGE
    m_rightBackMotor.configNeutralDeadband(0.0); //Ensures motors run at the same speed

    /* Config PID Slot 1 (Encoder) */
    //setDriveTrainGains(DriveConstants.DRIVE_DISTANCE_PID_LOOP_IDX, m_gains);
    m_leftFrontMotor.configMotionCruiseVelocity(15000, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configMotionAcceleration(6000, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configMotionCruiseVelocity(15000, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configMotionAcceleration(6000, DriveConstants.DRIVE_TIMEOUT_MS);
    //m_leftFrontMotor.setSelectedSensorPosition(0, DriveConstants.DRIVE_DISTANCE_PID_LOOP_IDX, DriveConstants.DRIVE_TIMEOUT_MS);
    //m_rightFrontMotor.setSelectedSensorPosition(0, DriveConstants.DRIVE_DISTANCE_PID_LOOP_IDX, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Config PID Slot 2 (Pigeon) */
    //m_leftFrontMotor.configRemoteFeedbackFilter(m_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, DriveConstants.DRIVE_GYRO_PID_LOOP_IDX, DriveConstants.DRIVE_TIMEOUT_MS);
    //m_leftFrontMotor.configAuxPIDPolarity();

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

      m_gains.kP = SmartDashboard.getNumber("P Value Drive", DriveConstants.DRIVE_GAINS.kP);
      m_gains.kI = SmartDashboard.getNumber("I Value Drive", DriveConstants.DRIVE_GAINS.kI);
      m_gains.kD = SmartDashboard.getNumber("D Value Drive", DriveConstants.DRIVE_GAINS.kD);
      m_gains.kF = SmartDashboard.getNumber("F Value Drive", DriveConstants.DRIVE_GAINS.kF);

      //setDriveTrainGains(DriveConstants.DRIVE_DISTANCE_PID_LOOP_IDX, m_gains);

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
  public void setDriveTrainGains(int slotPID, Gains gains){
    m_leftFrontMotor.selectProfileSlot(DriveConstants.DRIVE_SLOT_IDX, slotPID);
    m_leftFrontMotor.config_kF(DriveConstants.DRIVE_SLOT_IDX, gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.config_kP(DriveConstants.DRIVE_SLOT_IDX, gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.config_kI(DriveConstants.DRIVE_SLOT_IDX, gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.config_kD(DriveConstants.DRIVE_SLOT_IDX, gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);

    m_rightFrontMotor.selectProfileSlot(DriveConstants.DRIVE_SLOT_IDX, slotPID);
    m_rightFrontMotor.config_kF(DriveConstants.DRIVE_SLOT_IDX, gains.kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kP(DriveConstants.DRIVE_SLOT_IDX, gains.kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kI(DriveConstants.DRIVE_SLOT_IDX, gains.kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.config_kD(DriveConstants.DRIVE_SLOT_IDX, gains.kD, DriveConstants.DRIVE_TIMEOUT_MS);
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
