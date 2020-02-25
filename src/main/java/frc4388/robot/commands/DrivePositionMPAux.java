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

public class DrivePositionMPAux extends CommandBase {
  Drive m_drive;
  double m_cruiseVel;
  double m_rampDist;
  double m_targetPos;
  double m_currentVel;
  double m_currentPos;
  double m_targetGyro;
  double m_targetVel;
  double m_rampAcc;
  long m_startTime;
  long m_rampRate;

  /**
   * Creates a new DrivePositionMPAux.
   * 
   * @param subsystem The drive subsystem
   * @param cruiseVel The target velocity for the motors in units
   * @param rampDist  The distance before cruise velocity is reached in inches
   * @param rampRate  The time to reach the cruise velocity in seconds
   * @param targetPos The target position
   */
  public DrivePositionMPAux(Drive subsystem, double cruiseVel, double rampDist, float rampRate, double targetPos, double targetGyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_cruiseVel = cruiseVel * DriveConstants.TICKS_PER_INCH / 10;
    m_rampDist = rampDist * DriveConstants.TICKS_PER_INCH;
    m_rampRate = (long) rampRate * 1000;
    m_targetPos = targetPos * DriveConstants.TICKS_PER_INCH;
    //m_targetGyro = targetGyro * DriveConstants.TICKS_PER_GYRO_REV / 360;
    m_targetGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentVel = m_drive.m_rightFrontMotorVel;
    m_currentPos = m_drive.m_rightFrontMotorPos;
    m_targetPos = m_targetPos + m_currentPos;
    m_targetVel = m_currentVel;
    m_startTime = System.currentTimeMillis();
    m_rampAcc = (m_cruiseVel - m_currentVel) / m_rampRate;
  }

  // Called every m_isRamptime the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentVel = m_drive.m_rightFrontMotorVel;
    m_currentPos = m_drive.m_rightFrontMotorPos;
    if (System.currentTimeMillis() - m_startTime < m_rampRate) {
      // Ramping
      m_targetVel += m_rampAcc * m_drive.m_deltaTime;
      m_drive.runDriveVelocityPID(-m_targetVel, m_targetGyro);
    } else if (m_targetPos - m_currentPos > m_rampDist) {
      // Cruising
      m_drive.runDriveVelocityPID(-m_cruiseVel, m_targetGyro);
    } else {
      // Deramp PID
      m_drive.runDrivePositionPID(-m_targetPos, m_targetGyro);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_currentPos - m_targetPos <= 0.5f * DriveConstants.TICKS_PER_INCH) {
      return true;
    }
    return false;
  }
}
