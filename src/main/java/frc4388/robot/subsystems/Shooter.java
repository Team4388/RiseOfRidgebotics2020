/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.controller.XboxController;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  public static final WPI_TalonFX m_shooterTalon = new WPI_TalonFX(ShooterConstants.SHOOTER_CAN_ID);

  public Shooter() {

    m_shooterTalon.configFactoryDefault();

    m_shooterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTalon.configNominalOutputForward(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.configNominalOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.configPeakOutputForward(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.configPeakOutputReverse(-1, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTalon.selectProfileSlot(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_PID_LOOP_IDX);
    m_shooterTalon.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_GAINS.kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_GAINS.kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_GAINS.kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_GAINS.kD, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTalon.configMotionCruiseVelocity(15000, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterTalon.configMotionAcceleration(6000, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTalon.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try {
    
      SmartDashboard.putNumber("P Value Shooter", ShooterConstants.SHOOTER_GAINS.kP);
      SmartDashboard.putNumber("I Value Shooter", ShooterConstants.SHOOTER_GAINS.kI);
      SmartDashboard.putNumber("D Value Shooter", ShooterConstants.SHOOTER_GAINS.kD);
      SmartDashboard.putNumber("F Value Shooter", ShooterConstants.SHOOTER_GAINS.kF);

  } catch (Exception e) {

    System.err.println("The programming team failed successfully in the Shooter Subsystem.");
    
  }

  }

  public void getTargetPos() {
    double targetPos = XboxController.RIGHT_Y_AXIS * 2048 * 10.0;
    m_shooterTalon.set(ControlMode.MotionMagic, targetPos);
  }

}
