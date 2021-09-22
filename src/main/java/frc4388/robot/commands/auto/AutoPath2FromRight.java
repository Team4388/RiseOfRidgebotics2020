/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.commands.drive.GotoCoordinatesRobotRelative;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath2FromRight extends SequentialCommandGroup {
  Drive m_drive;
  Pneumatics m_pneumatics;

  /**
   * Creates a new AutoPath2FromRight.
   */
  public AutoPath2FromRight(Drive subsystem, Pneumatics subsystem2) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    m_drive = subsystem;
    m_pneumatics = subsystem2;
    
    addCommands(  new Wait(m_drive, 0),
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 77),
                  //Start Intake Ball 1
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 8),
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 28),
                  //Start Intake Ball 2
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 8),
                  //Shoot 5 Balls
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 28),
                  //Start Intake Ball 6 (Ball 1 second round)
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 8),
                  //Move to 7th Ball
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 86.7, -64.11, -180),
                  //Move to 8th Ball
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, -6.34, 15.31, 90),
                  //Move to 9th Ball
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 7.11, 24.41, 0),
                  //Move to 10th Ball
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, -6.34, 13.30),
                  //Shoot 5 more Balls (Total 10 Ball Autonomous Path)
                  new Wait(m_drive, 0)
                  );
  }
}
