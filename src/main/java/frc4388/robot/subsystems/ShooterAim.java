/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ShooterConstants;

public class ShooterAim extends SubsystemBase {
  public CANSparkMax m_angleAdjustMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);
  public CANSparkMax m_shooterRotateMotor = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID, MotorType.kBrushless);

  public static Gains m_shooterTurretGains = ShooterConstants.SHOOTER_TURRET_GAINS;

    // Configure PID Controllers
    CANPIDController m_angleAdjustPIDController = m_angleAdjustMotor.getPIDController();
    CANPIDController m_shooterRotatePIDController = m_shooterRotateMotor.getPIDController();
  
    CANEncoder m_angleEncoder = m_angleAdjustMotor.getEncoder();
    CANEncoder m_shooterRotateEncoder = m_shooterRotateMotor.getEncoder();
  /**
   * Creates a new ShooterAim.
   */
  public ShooterAim() {
    resetGyroAngleAdj();
    resetGyroShooterRotate();
    m_shooterRotateMotor.setIdleMode(IdleMode.kBrake);
    m_angleAdjustMotor.setIdleMode(IdleMode.kBrake);
  }

  public void runShooterWithInput(double input) {
    m_shooterRotateMotor.set(input);
  }

  /* Angle Adjustment PID Control */
  public void runAngleAdjustPID(double targetAngle)
  {
    // Set PID Coefficients
    m_angleAdjustPIDController.setP(m_shooterTurretGains.m_kP);
    m_angleAdjustPIDController.setI(m_shooterTurretGains.m_kI);
    m_angleAdjustPIDController.setD(m_shooterTurretGains.m_kD);
    m_angleAdjustPIDController.setIZone(m_shooterTurretGains.m_kIzone);
    m_angleAdjustPIDController.setFF(m_shooterTurretGains.m_kF);
    m_angleAdjustPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_shooterTurretGains.m_kPeakOutput); 

    // Convert input angle in degrees to rotations of the motor
    targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;

    m_angleAdjustPIDController.setReference(targetAngle, ControlType.kPosition);
  }

  /* Rotate Shooter PID Control */
  public void runshooterRotatePID(double targetAngle)
  {
    // Set PID Coefficients
    m_shooterRotatePIDController.setP(m_shooterTurretGains.m_kP);
    m_shooterRotatePIDController.setI(m_shooterTurretGains.m_kI);
    m_shooterRotatePIDController.setD(m_shooterTurretGains.m_kD);
    m_shooterRotatePIDController.setFF(m_shooterTurretGains.m_kF);
    m_shooterRotatePIDController.setIZone(m_shooterTurretGains.m_kIzone);
    m_shooterRotatePIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_shooterTurretGains.m_kPeakOutput); 

    // Convert input angle in degrees to rotations of the motor
    targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;

    m_shooterRotatePIDController.setReference(targetAngle, ControlType.kPosition);
  }


    /* For Testing Purposes, reseting gyro for angle adjuster */
    public void resetGyroAngleAdj()
    {
      m_angleEncoder.setPosition(0);
    }
  
    /* For Testing Purposes, reseting gyro for shooter rotation */
    public void resetGyroShooterRotate()
    {
      m_shooterRotateEncoder.setPosition(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
