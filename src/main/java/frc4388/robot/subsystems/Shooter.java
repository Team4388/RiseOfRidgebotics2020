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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  public WPI_TalonFX m_shooterFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_ID);
  public CANSparkMax m_angleAdjustMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);
  public CANSparkMax m_shooterRotateMotor = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID, MotorType.kBrushless);

  public static Gains m_shooterGains = ShooterConstants.SHOOTER_GAINS;

  // Configure PID Controllers
  CANPIDController m_angleAdjustPIDController = m_angleAdjustMotor.getPIDController();
  CANPIDController m_shooterRotatePIDController = m_shooterRotateMotor.getPIDController();

  CANEncoder m_angleEncoder = m_angleAdjustMotor.getEncoder();
  CANEncoder m_shooterRotateEncoder = m_shooterRotateMotor.getEncoder();

  double velP;
  /**
   * Creates a new Shooter subsystem.
   */
  public Shooter() {
    m_shooterFalcon.configFactoryDefault();

    m_shooterFalcon.setNeutralMode(NeutralMode.Coast);

    m_shooterFalcon.setInverted(true);
    
    setShooterGains();

    m_shooterFalcon.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    m_shooterFalcon.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    int closedLoopTimeMs = 1;
    m_shooterFalcon.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.configClosedLoopPeriod(1, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    m_shooterFalcon.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.m_kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.m_kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.m_kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterGains.m_kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
  }
  /**
   * Runs drum shooter velocity PID.
   * @param falcon Motor to use
   * @param targetVel Target velocity to run motor at
   */
  public void runDrumShooterVelocityPID(double targetVel, double actualVel) {
    velP = actualVel/targetVel; //Percent of target
    double runSpeed = actualVel + (12000*velP); //Ramp up equation
    //if (runSpeed > targetVel) {runSpeed = targetVel;}
    SmartDashboard.putNumber("shoot", actualVel);
    runSpeed = runSpeed/targetVel; //Convert to percent
    if (actualVel < targetVel - 1000){ //PID Based on ramp up amount
      m_shooterFalcon.set(TalonFXControlMode.PercentOutput, runSpeed);
    }
    else{ //PID Based on targetVel
      m_shooterFalcon.set(TalonFXControlMode.Velocity, targetVel); //Init PID
    }
  }

  /* Angle Adjustment PID Control */
  public void runAngleAdjustPID(double targetAngle, double kP, double kI, double kD, double kIz, double kF, double kmaxOutput, double kminOutput)
  {
    // Set PID Coefficients
    m_angleAdjustPIDController.setP(kP);
    m_angleAdjustPIDController.setI(kI);
    m_angleAdjustPIDController.setD(kD);
    m_angleAdjustPIDController.setIZone(kIz);
    m_angleAdjustPIDController.setFF(kF);
    m_angleAdjustPIDController.setOutputRange(kminOutput, kmaxOutput); 

    // Convert input angle in degrees to rotations of the motor
    targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;

    m_angleAdjustPIDController.setReference(targetAngle, ControlType.kPosition);
  }

  /* Rotate Shooter PID Control */
  public void runshooterRotatePID(double targetAngle, double kP, double kI, double kD, double kIz, double kF, double kmaxOutput, double kminOutput)
  {
    // Set PID Coefficients
    m_shooterRotatePIDController.setP(kP);
    m_shooterRotatePIDController.setI(kI);
    m_shooterRotatePIDController.setD(kD);
    m_shooterRotatePIDController.setIZone(kIz);
    m_shooterRotatePIDController.setFF(kF);
    m_shooterRotatePIDController.setOutputRange(kminOutput, kmaxOutput); 

    // Convert input angle in degrees to rotations of the motor
    targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;

    m_shooterRotatePIDController.setReference(targetAngle, ControlType.kPosition);
  }
}