/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;

public class SkipSong extends CommandBase {
  Drive m_drive;
  int m_index;
  
  /**
   * Creates a new SkipSong.
   */
  public SkipSong(Drive m_robotDrive, int index) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = m_robotDrive;
    m_index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String[] songs = m_drive.songsStrings;
    String song = m_drive.m_currentSong;

    for (int i = 0; i < songs.length; i++) {
      if (songs[i] == song) {
        m_drive.selectSong(songs[i + m_index]);
        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
