/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  WPI_TalonFX m_shooterFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_ID);

  public static Gains m_shooterGains = ShooterConstants.SHOOTER_GAINS;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_shooterFalcon.configFactoryDefault();

    m_shooterFalcon.setNeutralMode(NeutralMode.Coast);

    m_shooterFalcon.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs drum shooter motor.
   * @param speed
   */
  public void runDrumShooter(double speed) {
    m_shooterFalcon.set(speed);
  }

  /**
   * Configures gains for shooter PID.
   */
  public void setShooterGains() {
    m_shooterFalcon.selectProfileSlot(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_PID_LOOP_IDX);
    m_shooterFalcon.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
  }
}
