/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  public WPI_TalonFX m_shooterFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_ID);

  private double m_targetVel = 2300;

  public static Gains m_shooterGains = ShooterConstants.SHOOTER_GAINS;

  double velP;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_shooterFalcon.configFactoryDefault();

    m_shooterFalcon.setNeutralMode(NeutralMode.Coast);

    m_shooterFalcon.setInverted(true);
    
    setShooterGains();
    
    m_shooterFalcon.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    int closedLoopTimeMs = 1;
    m_shooterFalcon.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.configClosedLoopPeriod(1, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);

    SmartDashboard.putNumber("Shooter Velocity Target", m_targetVel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_targetVel = SmartDashboard.getNumber("Shooter Velocity Target", m_targetVel);
    SmartDashboard.putNumber("Shooter Velocity", m_shooterFalcon.getSelectedSensorVelocity()*600/ShooterConstants.ENCODER_TICKS_PER_REV);
  }

  /**
   * Runs drum shooter motor.
   * @param speed Speed to set the motor at
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
  /**
   * Runs drum shooter velocity PID.
   * @param falcon Motor to use
   * @param targetVel Target velocity to run motor at
   */
  public void runDrumShooterVelocityPID(double targetVel, double actualVel) {
    velP = actualVel/targetVel;
    if(velP < 0.1){
      velP = 0.1;
    }
    double runSpeed = velP*(1-velP);
    //System.err.println(runSpeed);
    m_shooterFalcon.set(TalonFXControlMode.PercentOutput, runSpeed/*ShooterConstants.ENCODER_TICKS_PER_REV/600*/);
  }
}