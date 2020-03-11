/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  Intake m_intake;
  /**
   * Creates a new RunIntake.
   */
  public RunIntake(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = subsystem;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(IntakeConstants.INTAKE_SPEED);
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
