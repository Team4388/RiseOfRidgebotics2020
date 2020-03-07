/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;

public class PlaySongDrive extends CommandBase {
  private Drive m_drive;
  
  /**
   * Creates a new PlaySongDrive.
   */
  public PlaySongDrive(Drive subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.m_rightFrontMotor.set(0);
    m_drive.m_leftFrontMotor.set(0);
    m_drive.m_rightBackMotor.set(0);
    m_drive.m_leftBackMotor.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.playSong();
    //System.err.println("Playing " + m_drive.m_orchestra.isPlaying());
    //m_drive.m_driveTrain.feedWatchdog();
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
