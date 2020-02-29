/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath1FromCenter extends SequentialCommandGroup {
  Drive m_drive;

  /**
   * Creates a new AutoPath1FromCenter.
   */
  public AutoPath1FromCenter(Drive subsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    m_drive = subsystem;

    addCommands(  new Wait(m_drive, 0, 1), 
                  //shoot pre-loaded 3 balls
                  new GotoCoordinates(m_drive, 75, 44, -90),
                  //Start Intake Ball 1
                  new GotoCoordinates(m_drive, 0, 12),
                  new GotoCoordinates(m_drive, 0, 28),
                  //Start Intake Ball 2
                  new GotoCoordinates(m_drive, 0, 8),
                  new GotoCoordinates(m_drive, 0, 28),
                  //Start Intake Ball 3
                  new GotoCoordinates(m_drive, 0, 8),
                  new Wait(m_drive, 0, 2)
                  //Shoot 3 Balls
                  );
  }
}
