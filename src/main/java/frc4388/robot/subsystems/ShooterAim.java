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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ShooterConstants;

public class ShooterAim extends SubsystemBase {
  public CANSparkMax m_shooterRotateMotor = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID, MotorType.kBrushless);
  public static Gains m_shooterTurretGains = ShooterConstants.SHOOTER_TURRET_GAINS;
  CANDigitalInput m_shooterRightLimit, m_shooterLeftLimit;

    // Configure PID Controllers
    CANPIDController m_shooterRotatePIDController = m_shooterRotateMotor.getPIDController();
    public CANEncoder m_shooterRotateEncoder = m_shooterRotateMotor.getEncoder();


  /**
   * Creates a subsytem for the turret aiming
   */
  public ShooterAim() {
    //resetGyroShooterRotate();
    m_shooterRotateMotor.setIdleMode(IdleMode.kBrake);

    m_shooterLeftLimit = m_shooterRotateMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_shooterRightLimit = m_shooterRotateMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_shooterRightLimit.enableLimitSwitch(true);
    m_shooterLeftLimit.enableLimitSwitch(true);

    m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_shooterRotateMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.TURRET_RIGHT_SOFT_LIMIT);
    m_shooterRotateMotor.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.TURRET_LEFT_SOFT_LIMIT);
  }

  public void runShooterWithInput(double input) {
    m_shooterRotateMotor.set(input*ShooterConstants.TURRET_SPEED_MULTIPLIER);
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

    public void resetGyroShooterRotate()
    {
        m_shooterRotateEncoder.setPosition(0);
    }

    public double getShooterRotatePosition(){
      return m_shooterRotateMotor.getEncoder().getPosition();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
