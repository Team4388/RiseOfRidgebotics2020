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
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;
import frc4388.utility.ShooterTables;
import frc4388.utility.Trims;
import frc4388.utility.controller.IHandController;

public class Shooter extends SubsystemBase {

  public WPI_TalonFX m_shooterFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_ID);

  public static Gains m_drumShooterGains = ShooterConstants.DRUM_SHOOTER_GAINS;
  public static Shooter m_shooter;
  public static IHandController m_controller;
  
  double velP;
  double input;

  public ShooterTables m_shooterTable;
  
  public boolean m_isDrumReady = false;
  public double m_fireVel;

  public Trims shooterTrims;

  public ShooterHood m_shooterHoodSubsystem;
  public ShooterAim m_shooterAimSubsystem;
  
  /*
   * Creates a new Shooter subsystem, with the drum shooter and the angle adjsuter.
   */
  public Shooter() {
    //Testing purposes reseting gyros
    //resetGyroAngleAdj();
    shooterTrims = new Trims(0, 0);
    //SmartDashboard.putNumber("Velocity Target", 10000);
    //SmartDashboard.putNumber("Angle Target", 3);

    m_shooterFalcon.configFactoryDefault();
    m_shooterFalcon.setNeutralMode(NeutralMode.Coast);
    m_shooterFalcon.setInverted(true);
    m_shooterFalcon.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    //m_shooterFalcon.configClosedloopRamp(1.0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    setShooterGains();

    m_shooterFalcon.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    m_shooterFalcon.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    
    int closedLoopTimeMs = 1;
    m_shooterFalcon.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);

    m_shooterTable = new ShooterTables();

    m_shooterFalcon.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);

    //SmartDashboard.putNumber("CSV 10", m_shooterTable.getVelocity(10));
    //SmartDashboard.putNumber("CSV 200", m_shooterTable.getVelocity(200));
    //SmartDashboard.putNumber("CSV 250", m_shooterTable.getVelocity(250));
    //SmartDashboard.putNumber("CSV 500", m_shooterTable.getVelocity(500));

    //SmartDashboard.putNumber("CSV A -30", m_shooterTable.getAngleDisplacement(-30));
    //SmartDashboard.putNumber("CSV A 10", m_shooterTable.getAngleDisplacement(10));
    //SmartDashboard.putNumber("CSV A 5", m_shooterTable.getAngleDisplacement(5));
    //SmartDashboard.putNumber("CSV A 30", m_shooterTable.getAngleDisplacement(30));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try{
      SmartDashboard.putNumber("Drum Velocity", m_shooterFalcon.getSelectedSensorVelocity());

      SmartDashboard.putNumber("Drum Velocity CSV", m_fireVel);

      SmartDashboard.putNumber("Shooter Temp C", m_shooterFalcon.getTemperature());

      SmartDashboard.putNumber("Shooter Current", m_shooterFalcon.getSupplyCurrent());

      SmartDashboard.putBoolean("Drum Ready" , m_isDrumReady);
    }

    catch(Exception e)
    {
      
    }
  }

  /**
   * Passes subsystem needed.
   * @param subsystem Subsystem needed.
   */
  public void passRequiredSubsystem(ShooterHood subsystem0, ShooterAim subsystem1) {
    m_shooterHoodSubsystem = subsystem0;
    m_shooterAimSubsystem = subsystem1;
  }

  public double addFireVel() {
    return m_fireVel;
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

  /**
   * Runs drum shooter velocity PID.
   * @param targetVel Target velocity to run motor at
   */
  public void runDrumShooterVelocityPID(double targetVel) {
    //System.out.println("Target Velocity" + targetVel);
    m_shooterFalcon.set(TalonFXControlMode.Velocity, targetVel); //Init PID
  }
}