/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public WPI_TalonSRX m_talon1 = new WPI_TalonSRX(ElevatorConstants.TALON_1);
  public WPI_TalonSRX m_talon2 = new WPI_TalonSRX(ElevatorConstants.TALON_2);

  /**
   * Creates a new Elevator.
   */
  public Elevator() {

    m_talon1.configFactoryDefault();
    m_talon2.configFactoryDefault();

    m_talon2.follow(m_talon1);

    m_talon1.setNeutralMode(NeutralMode.Brake);
    m_talon2.setNeutralMode(NeutralMode.Brake);

    m_talon1.setInverted(false);
    m_talon2.setInverted(false);
    m_talon1.setInverted(InvertType.FollowMaster);
    m_talon2.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 public void moveElevator(double speed){
   m_talon1.set(speed);
   m_talon2.set(speed);

 }
 }
  
