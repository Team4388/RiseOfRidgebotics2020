/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wait extends CommandBase {

  long m_startTime;
  long m_waitTime;
  long m_currentTime;
  SubsystemBase m_subsystem;
  int m_waitNum;

  int counter  = 0;

  /**
   * Creates a new WaitCommand.
   */
  public Wait(SubsystemBase subsystem, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_waitTime = (long) (seconds * 1000);
    m_subsystem = subsystem;
   // m_waitNum = waitNum;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentTime = System.currentTimeMillis();
    m_startTime = m_currentTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (counter == 0) {
      //SmartDashboard.putNumber("Wait Coordinates" + m_waitNum, m_currentTime);
    }

    m_currentTime = System.currentTimeMillis();
    //SmartDashboard.putNumber("Time Difference for Wait", (m_currentTime - m_startTime));

    counter ++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((m_currentTime - m_startTime) >= m_waitTime) {
      return true;
    } else {
      return false;
    }
  }
}
