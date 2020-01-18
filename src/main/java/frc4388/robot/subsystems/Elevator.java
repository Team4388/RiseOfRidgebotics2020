/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  
  public WPI_TalonSRX m_talon1 = new WPI_TalonSRX(ElevatorConstants.TALON_1);
  public WPI_TalonSRX m_talon2 = new WPI_TalonSRX(ElevatorConstants.TALON_2);

  public static Gains m_elevatorGains = ElevatorConstants.ELEVATOR_GAINS;
  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    //resets motors
    m_talon1.configFactoryDefault();
    m_talon2.configFactoryDefault();

    //config following settings
    m_talon2.follow(m_talon1);

    m_talon1.setNeutralMode(NeutralMode.Brake);
    m_talon2.setNeutralMode(NeutralMode.Brake);

    m_talon1.setInverted(false);
    m_talon2.setInverted(false);
    m_talon1.setInverted(InvertType.FollowMaster);
    m_talon2.setInverted(InvertType.FollowMaster);

    setElevatorGains();

    m_talon1.setSelectedSensorPosition(0, ElevatorConstants.ELEVATOR_PID_LOOP_IDX, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon2.setSelectedSensorPosition(0, ElevatorConstants.ELEVATOR_PID_LOOP_IDX, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
  
    int closedLoopTimeMs = 1;
    m_talon1.configClosedLoopPeriod(0, closedLoopTimeMs, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon1.configClosedLoopPeriod(1, closedLoopTimeMs, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void moveElevator(double speed) {
    m_talon1.set(speed);
    m_talon2.set(speed);
  }
  public void setElevatorGains(){
    m_talon1.selectProfileSlot(ElevatorConstants.ELEVATOR_SLOT_IDX, ElevatorConstants.ELEVATOR_PID_LOOP_IDX);
    m_talon1.config_kF(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kF, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon1.config_kP(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kP, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon1.config_kI(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kI, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon1.config_kD(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kD, ElevatorConstants.ELEVATOR_TIMEOUT_MS);

    m_talon2.selectProfileSlot(ElevatorConstants.ELEVATOR_SLOT_IDX, ElevatorConstants.ELEVATOR_PID_LOOP_IDX);
    m_talon2.config_kF(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kF, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon2.config_kP(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kP, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon2.config_kI(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kI, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
    m_talon2.config_kD(ElevatorConstants.ELEVATOR_SLOT_IDX, m_elevatorGains.kD, ElevatorConstants.ELEVATOR_TIMEOUT_MS);
  }
  public void runElevatorPositionPID(WPI_TalonSRX talon, double targetPos) {
    talon.set(ControlMode.Position, targetPos);
  }
}
