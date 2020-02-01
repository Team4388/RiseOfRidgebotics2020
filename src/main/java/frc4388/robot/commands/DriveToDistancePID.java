/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;

public class DriveToDistancePID extends CommandBase {
  Drive m_drive;
  double m_distance;
  double m_leftTarget;
  double m_rightTarget;
  
  /**
   * Creates a new DriveToDistancePID.
   * @param subsystem drive subsystem
   * @param distance distance to travel in inches
   */
  public DriveToDistancePID(Drive subsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_distance = distance * DriveConstants.TICKS_PER_INCH;
    addRequirements(m_drive);
    SmartDashboard.putNumber("Distance Target Inches", distance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftTarget = m_drive.m_leftFrontMotor.getActiveTrajectoryPosition() + m_distance;
    m_rightTarget = -(m_drive.m_rightFrontMotor.getActiveTrajectoryPosition() + m_distance);
    SmartDashboard.putNumber("Distance Target Ticks", m_distance);
    SmartDashboard.putNumber("Left Target Ticks", m_leftTarget);
    SmartDashboard.putNumber("Right Target Ticks", m_rightTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drive.runPositionPID(m_drive.m_leftFrontMotor, m_leftTarget);
    //m_drive.runPositionPID(m_drive.m_rightFrontMotor, m_rightTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_drive.m_leftFrontMotor.getActiveTrajectoryPosition() - m_leftTarget) < 100){
      return true;
    } else {
      return false;
    }
  }
}
