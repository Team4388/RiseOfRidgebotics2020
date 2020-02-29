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
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.ShooterTables;
import frc4388.utility.controller.IHandController;

public class Shooter extends SubsystemBase {

  public WPI_TalonFX m_shooterFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_ID);
  public CANSparkMax m_angleAdjustMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);

  CANPIDController m_angleAdjustPIDController = m_angleAdjustMotor.getPIDController();
  CANEncoder m_angleEncoder = m_angleAdjustMotor.getEncoder();

  public static Gains m_drumShooterGains = ShooterConstants.DRUM_SHOOTER_GAINS;
  public static Gains m_angleAdjustGains = ShooterConstants.SHOOTER_ANGLE_GAINS;
  public static Shooter m_shooter;
  public static IHandController m_controller;
  
  double velP;
  double input;

  ShooterTables m_shooterTable;
  
  public boolean velReached;
  public double m_fireVel;
  public double m_fireAngle;
  CANDigitalInput m_hoodUpLimit, m_hoodDownLimit;
  
  /*
   * Creates a new Shooter subsystem.
   */
  public Shooter() {
    //Testing purposes reseting gyros
    resetGyroAngleAdj();
    m_shooterFalcon.configFactoryDefault();
    m_shooterFalcon.setNeutralMode(NeutralMode.Coast);
    m_shooterFalcon.setInverted(false);
    m_angleAdjustMotor.setIdleMode(IdleMode.kBrake);
    m_shooterFalcon.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    setShooterGains();

    m_shooterFalcon.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    m_shooterFalcon.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    int closedLoopTimeMs = 1;
    m_shooterFalcon.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.configClosedLoopPeriod(1, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTable = new ShooterTables();

    SmartDashboard.putNumber("CSV 10", m_shooterTable.getVelocity(10));
    SmartDashboard.putNumber("CSV 200", m_shooterTable.getVelocity(200));
    SmartDashboard.putNumber("CSV 250", m_shooterTable.getVelocity(250));
    SmartDashboard.putNumber("CSV 500", m_shooterTable.getVelocity(500));

    SmartDashboard.putNumber("CSV A -30", m_shooterTable.getAngleDisplacement(-30));
    SmartDashboard.putNumber("CSV A 10", m_shooterTable.getAngleDisplacement(10));
    SmartDashboard.putNumber("CSV A 5", m_shooterTable.getAngleDisplacement(5));
    SmartDashboard.putNumber("CSV A 30", m_shooterTable.getAngleDisplacement(30));



    m_hoodUpLimit = m_angleAdjustMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_hoodDownLimit = m_angleAdjustMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_hoodUpLimit.enableLimitSwitch(true);
    m_hoodDownLimit.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", m_shooterFalcon.getSelectedSensorVelocity()*600/ShooterConstants.ENCODER_TICKS_PER_REV);
  }

  public double addFireVel() {
    return m_fireVel;
  }
  public double addFireAngle() {
    return m_fireAngle;
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
    m_shooterFalcon.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalcon.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
  }

  /* Angle Adjustment PID Control */
  public void runAngleAdjustPID(double mmtargetAngle)
  {
    double targetAngle = addFireAngle();
    // Set PID Coefficients
    m_angleAdjustPIDController.setP(m_angleAdjustGains.m_kP);
    m_angleAdjustPIDController.setI(m_angleAdjustGains.m_kI);
    m_angleAdjustPIDController.setD(m_angleAdjustGains.m_kD);
    m_angleAdjustPIDController.setIZone(m_angleAdjustGains.m_kIzone);
    m_angleAdjustPIDController.setFF(m_angleAdjustGains.m_kF);
    m_angleAdjustPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_angleAdjustGains.m_kPeakOutput); 

    // Convert input angle in degrees to rotations of the motor
    targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;

    m_angleAdjustPIDController.setReference(targetAngle, ControlType.kPosition);
  }

  /**
   * Runs drum shooter velocity PID.
   * @param falcon Motor to use
   * @param targetVel Target velocity to run motor at
   */
  public void runDrumShooterVelocityPID(double targetVel, double actualVel) {
    SmartDashboard.putNumber("shoot", actualVel);
    if (actualVel < targetVel - 1000){
      m_shooterFalcon.set(TalonFXControlMode.PercentOutput, 1);//Ramp up
    }
    else{ //PID Based on targetVel
      m_shooterFalcon.set(TalonFXControlMode.Velocity, targetVel); //Init PID
    }
    m_shooterFalcon.feed();
  }

  public void resetGyroAngleAdj(){
      m_angleEncoder.setPosition(0);
  }
}