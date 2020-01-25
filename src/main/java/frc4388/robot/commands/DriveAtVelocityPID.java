/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;

public class DriveAtVelocityPID extends CommandBase {
  Drive m_drive;
  double m_targetVel;
  double m_leftTarget;
  double m_rightTarget;
  double m_copiedTargetVel;
  /**
   * Creates a new DriveAtVelocityPID.
   * @param subsystem drive subsystem
   * @param distance target velocity in inches/second
   */
  public DriveAtVelocityPID(Drive subsystem, double targetVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_targetVel = targetVel * DriveConstants.TICKS_PER_INCH/DriveConstants.SECONDS_TO_TICK_TIME;
    m_copiedTargetVel = targetVel;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftTarget =  m_targetVel;
    m_rightTarget = m_targetVel;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.runVelocityPID(m_targetVel);

    SmartDashboard.putNumber("Input Target Velocity", m_copiedTargetVel);
    SmartDashboard.putNumber("Output Target Velocity", m_targetVel);
    //m_drive.runVelocityPID(m_leftTarget);
    //m_drive.m_leftFrontMotor.follow(m_drive.m_rightFrontMotor, FollowerType.PercentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
