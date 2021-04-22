/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc4388.robot.subsystems.LimeLight;

public class ExecuteCommand extends CommandBase {
  /**
   * Creates a new ExecuteCommand.
   */
  RamseteCommand[] m_paths;
  LimeLight m_limeLight;
  public ExecuteCommand(LimeLight limeLight, RamseteCommand[] paths) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limeLight = limeLight;
    m_paths = paths;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String gsPath = m_limeLight.galacticSearchPath;
    switch (gsPath)
    {
      case "A_RED":
        new RunPath(m_paths[0]);
        break;
      case "A_BLUE":
        new RunPath(m_paths[1]);
        break;
      case "B_RED":
        new RunPath(m_paths[2]);
        break;
      case "B_BLUE":
        new RunPath(m_paths[3]);
        break;
      case "test":
        new RunPath(m_paths[0]);
        break;
    }
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
