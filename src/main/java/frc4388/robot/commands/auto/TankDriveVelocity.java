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

  double m_targetTime;
  double m_firstTimeSec;
  double m_currentTimeSec;
  double m_diffSec;

  /**
   * Creates a new TankDriveVelocity.
   */
  public TankDriveVelocity(Drive subsystem, double leftTargetVel, double rightTargetVel, double targetTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_leftTargetVel = leftTargetVel;
    m_rightTargetVel = rightTargetVel;
    m_targetTime = targetTime;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_firstTimeSec = (System.currentTimeMillis() / 1000);
    m_diffSec = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentTimeSec = (System.currentTimeMillis() / 1000);
    m_diffSec = m_currentTimeSec - m_firstTimeSec;
    
    if (m_diffSec < m_targetTime) {
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
    if (m_diffSec >= m_targetTime) {
      return true;
    }
    return false;
  }
}
