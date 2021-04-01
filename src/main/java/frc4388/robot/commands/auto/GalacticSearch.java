/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.commands.intake.RunIntake;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
  /**
   * Creates a new GalacticSearch.
   */
  public GalacticSearch(LimeLight m_limeLight, Intake m_intake, RamseteCommand[] paths) {
    if (m_limeLight.galacticSearchPath == "A_RED")
    {
      addCommands(new SequentialCommandGroup(paths[0], new RunIntake(m_intake)));
    }
    else if (m_limeLight.galacticSearchPath == "A_BLUE")
    {
      addCommands(new SequentialCommandGroup(paths[1], new RunIntake(m_intake)));
    }
    else if (m_limeLight.galacticSearchPath == "B_RED")
    {
      addCommands(new SequentialCommandGroup(paths[2], new RunIntake(m_intake)));
    }
    else if (m_limeLight.galacticSearchPath == "B_BLUE")
    {
      addCommands(new SequentialCommandGroup(paths[3], new RunIntake(m_intake)));
    }
  }
}
