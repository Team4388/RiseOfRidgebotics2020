/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;

public class DriveStraightAtVelocityPID extends CommandBase {
  Drive m_drive;
  double m_targetVel;
  double m_targetGyro;
  /**
   * Creates a new DriveStraightAtVelocityPID.
   * @param subsystem The drive subsystem
   * @param targetVel The target velocity for the motors in units
   */
  public DriveStraightAtVelocityPID(Drive subsystem, double targetVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_targetVel = targetVel;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.err.println(m_drive.m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_TURN));
    m_drive.runDriveVelocityPID(m_targetVel, m_targetGyro);
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
