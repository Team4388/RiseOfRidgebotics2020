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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.controller.IHandController;

public class Shooter extends SubsystemBase {

  public WPI_TalonFX m_shooterFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_ID);
  public CANSparkMax m_angleAdjustMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);
  public CANSparkMax m_shooterRotateMotor = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID, MotorType.kBrushless);


  public static Gains m_shooterTurretGains = ShooterConstants.SHOOTER_TURRET_GAINS;
  public static Gains m_drumShooterGains = ShooterConstants.DRUM_SHOOTER_GAINS;
  public static Shooter m_shooter;
  public static IHandController m_controller;


  // Configure PID Controllers
  CANPIDController m_angleAdjustPIDController = m_angleAdjustMotor.getPIDController();
  CANPIDController m_shooterRotatePIDController = m_shooterRotateMotor.getPIDController();

  CANEncoder m_angleEncoder = m_angleAdjustMotor.getEncoder();
  CANEncoder m_shooterRotateEncoder = m_shooterRotateMotor.getEncoder();

  double velP;
  double input;
  
  /*
   * Creates a new Shooter subsystem.
   */
  public Shooter() {
    //Testing purposes reseting gyros
    resetGyroAngleAdj();
    resetGyroShooterRotate();

    m_shooterFalcon.configFactoryDefault();
    m_shooterRotateMotor.setIdleMode(IdleMode.kBrake);
    m_shooterFalcon.setNeutralMode(NeutralMode.Coast);
    m_shooterFalcon.setInverted(false);
    
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
    m_shooterFalcon.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterTurretGains.m_kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterTurretGains.m_kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterTurretGains.m_kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_shooterTurretGains.m_kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
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


  public void runShooterWithInput(IHandController controller) {
    /* m_controller = controller;
     input = controller.getLeftXAxis();
     * System.err.println(input);
     * m_shooterFalcon.set(TalonFXControlMode.PercentOutput, 0.3);
     */
    input = controller.getLeftXAxis();
    System.err.println(input);
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
}