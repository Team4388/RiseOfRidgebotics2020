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
public class AutoPath1FromCenter extends SequentialCommandGroup {
  Drive m_drive;
  Pneumatics m_pneumatics;

  /**
   * Creates a new AutoPath1FromCenter.
   */
  public AutoPath1FromCenter(Drive subsystem, Pneumatics subsystem2) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    m_drive = subsystem;
    m_pneumatics = subsystem2;

    addCommands(  new Wait(m_drive, 0, 1), 
                  //shoot pre-loaded 3 balls
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 75, 44, -90),
                  //Start Intake Ball 1
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 12),
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 28),
                  //Start Intake Ball 2
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 8),
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 28),
                  //Start Intake Ball 3
                  new GotoCoordinatesRobotRelative(m_drive, m_pneumatics, 0, 8),
                  new Wait(m_drive, 0, 2)
                  //Shoot 3 Balls
                  );
  }
}
