/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Climber;
import frc4388.utility.controller.IHandController;

public class RunClimberWithTriggers extends CommandBase {
  private Climber m_climber;
  private IHandController m_controller;

  /**
   * Uses input from opperator triggers to control climber motor
   * @param subsystem the climber subsystem
   * @param controller the driver controller
   */
  public RunClimberWithTriggers(Climber subsystem, IHandController controller) {
    m_climber = subsystem;
    m_controller = controller;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightTrigger = m_controller.getRightTriggerAxis();
    double leftTrigger = m_controller.getLeftTriggerAxis();
    double output = 0;
    if (rightTrigger < .5) {
      if(rightTrigger > leftTrigger) {
        output = rightTrigger;
      }
      if (leftTrigger > rightTrigger) {
        output = -leftTrigger;
        m_climber.shiftServo(true);
      }
    } else {
      output = rightTrigger;
    }
    m_climber.runClimber(output);
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
