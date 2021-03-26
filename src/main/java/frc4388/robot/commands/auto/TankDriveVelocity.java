/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;

public class TankDriveVelocity extends CommandBase {
  Drive m_drive;
  double m_leftTargetVel;
  double m_rightTargetVel;

  long m_targetTime;
  long m_firstTime;
  long m_currentTime;
  long m_diffTime;

  /**
   * Creates a new TankDriveVelocity.
   */
  public TankDriveVelocity(Drive subsystem, double leftTargetVel, double rightTargetVel, double targetTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_leftTargetVel = leftTargetVel;
    m_rightTargetVel = rightTargetVel;
    m_targetTime = (long) (targetTime * 1000);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_firstTime = System.currentTimeMillis();
    m_diffTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentTime = System.currentTimeMillis();
    m_diffTime = m_currentTime - m_firstTime;
    
    if (m_diffTime < m_targetTime) {
      m_drive.tankDriveVelocity(m_leftTargetVel, m_rightTargetVel);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_diffTime >= m_targetTime) {
      return true;
    }
    return false;
  }
}
