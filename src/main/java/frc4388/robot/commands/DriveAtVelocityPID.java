/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;

public class DriveAtVelocityPID extends CommandBase {
  Drive m_drive;
  double m_targetVel;
  double m_leftTarget;
  double m_rightTarget;
  /**
   * Creates a new DriveAtVelocityPID.
   */
  public DriveAtVelocityPID(Drive subsystem, double targetVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_targetVel = targetVel;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftTarget = m_targetVel;
    m_rightTarget = -m_targetVel;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.runVelocityPID(m_drive.m_leftFrontMotor, m_leftTarget);
    m_drive.runVelocityPID(m_drive.m_rightFrontMotor, m_rightTarget);
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
