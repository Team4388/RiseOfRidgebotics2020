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

public class DriveStraightToPositionPID extends CommandBase {
  Drive m_drive;
  double m_targetPosIn;
  double m_targetPosOut;
  double m_targetGyro;
  int i;
  
  /**
   * Creates a new DriveToDistancePID.
   * @param subsystem drive subsystem
   * @param targetPos distance to travel in inches
   */
  public DriveStraightToPositionPID(Drive subsystem, double targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_targetPosIn = targetPos * DriveConstants.TICKS_PER_INCH;
    addRequirements(m_drive);
    //SmartDashboard.putNumber("Distance Target Inches", targetPos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.err.println("PID START \n | \n |");
    m_targetGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    m_targetPosOut = m_targetPosIn + m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_PRIMARY);
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.err.println("| \n Sensor Pos \n" + m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_PRIMARY));
    //System.err.println("Sensor Error \n" + m_drive.m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_PRIMARY));
    //System.err.println("Sensor Target \n" + m_drive.m_rightFrontMotor.getClosedLoopTarget(DriveConstants.PID_PRIMARY));
    m_drive.runDrivePositionPID(m_targetPosOut, m_targetGyro);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs((int)m_drive.m_rightFrontMotor.getSelectedSensorVelocity(DriveConstants.PID_PRIMARY)) < 5 && i > 5){
      return true;
    } else {
      i++;
      //System.err.println(i);
      return false;
    }
  }
}
