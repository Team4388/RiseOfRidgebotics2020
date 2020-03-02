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
import frc4388.robot.subsystems.Pneumatics;

public class DriveStraightToPositionMM extends CommandBase {
  Drive m_drive;
  Pneumatics m_pneumatics;
  double m_targetPosIn;
  double m_targetPosOut;
  double m_targetGyro;
  boolean isGoneFast;
  int i;
  
  /**
   * Creates a new DriveToDistancePID.
   * @param subsystem drive subsystem
   * @param targetPos distance to travel in inches
   */
  public DriveStraightToPositionMM(Drive subsystem, Pneumatics subsystem2, double targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_pneumatics = subsystem2;
    try {
      if (m_pneumatics.m_isSpeedShiftHigh) {
        m_targetPosIn = targetPos * DriveConstants.TICKS_PER_INCH_HIGH * 2;
      } else {
        m_targetPosIn = targetPos * DriveConstants.TICKS_PER_INCH_LOW * 2;
      }
    } catch (Exception e) {
      System.err.println("Error In Motion Magic Switch Gains.");
    }
    addRequirements(m_drive);
    //SmartDashboard.putNumber("Distance Target Inches", targetPos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.err.println("PID START \n | \n |");
    m_targetGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    m_targetPosOut = m_targetPosIn + m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_PRIMARY);
    isGoneFast = false;
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.err.println("| \n Sensor Pos \n" + m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN));
    //System.err.println("Sensor Error \n" + m_drive.m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_TURN));
    //System.err.println("Sensor Target \n" + m_drive.m_rightFrontMotor.getClosedLoopTarget(DriveConstants.PID_TURN));
    m_drive.runMotionMagicPID(m_targetPosOut, m_targetGyro);
    SmartDashboard.putBoolean("MM Run", true);
    i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs((int)m_drive.m_rightFrontMotor.getSelectedSensorVelocity(DriveConstants.PID_PRIMARY)) < 5 && isGoneFast){
      SmartDashboard.putBoolean("MM Run", false);
      return true;
    } else {
      if ((m_drive.m_rightFrontMotor.getSelectedSensorVelocity(DriveConstants.PID_PRIMARY) > 100)) {
        isGoneFast = true;
      }
      return false;
    }
  }
}
