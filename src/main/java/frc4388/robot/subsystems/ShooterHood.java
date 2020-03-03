/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;
import frc4388.utility.controller.IHandController;

public class ShooterHood extends SubsystemBase {
  public Shooter m_shooterSubsystem;

  public CANSparkMax m_angleAdjustMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);

  public static Gains m_angleAdjustGains = ShooterConstants.SHOOTER_ANGLE_GAINS;
  
  public CANPIDController m_angleAdjustPIDController = m_angleAdjustMotor.getPIDController();
  public CANEncoder m_angleEncoder = m_angleAdjustMotor.getEncoder();

  public double m_fireAngle;
  public CANDigitalInput m_hoodUpLimit, m_hoodDownLimit;

  /**
   * Creates a new ShooterHood.
   */
  public ShooterHood() {
    m_angleAdjustMotor.setIdleMode(IdleMode.kBrake);

    m_hoodUpLimit = m_angleAdjustMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_hoodDownLimit = m_angleAdjustMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_hoodUpLimit.enableLimitSwitch(true);
    m_hoodDownLimit.enableLimitSwitch(true);

    m_angleAdjustMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_angleAdjustMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_angleAdjustMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.HOOD_UP_SOFT_LIMIT);
    m_angleAdjustMotor.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.HOOD_DOWN_SOFT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Fire Angle CSV", m_fireAngle);
  }

  /**
   * Passes subsystem needed.
   * @param subsystem Subsystem needed.
   */
  public void passRequiredSubsystem(Shooter subsystem) {
    m_shooterSubsystem = subsystem;
  }

  public double addFireAngle() {
    return m_fireAngle;
  }

  /* Angle Adjustment PID Control */
  public void runAngleAdjustPID(double targetAngle)
  {
    // Set PID Coefficients
    m_angleAdjustPIDController.setP(m_angleAdjustGains.m_kP);
    m_angleAdjustPIDController.setI(m_angleAdjustGains.m_kI);
    m_angleAdjustPIDController.setD(m_angleAdjustGains.m_kD);
    m_angleAdjustPIDController.setIZone(m_angleAdjustGains.m_kIzone);
    m_angleAdjustPIDController.setFF(m_angleAdjustGains.m_kF);
    m_angleAdjustPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_angleAdjustGains.m_kPeakOutput); 

    m_angleAdjustPIDController.setReference(targetAngle, ControlType.kPosition);
  }

  public void resetGyroAngleAdj(){
    m_angleEncoder.setPosition(0);
}

  public double getAnglePosition(){
    return m_angleEncoder.getPosition();
  }
}
