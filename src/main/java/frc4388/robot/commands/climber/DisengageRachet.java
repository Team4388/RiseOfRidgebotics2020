/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Climber;

public class DisengageRachet extends CommandBase {
  Climber m_climber;
  
  /**
   * Creates a new DisengageRachet command.
   */
  public DisengageRachet(Climber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.climberSafety) {
      m_climber.shiftServo(false);
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
