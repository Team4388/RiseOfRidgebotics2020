/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Intake;
import frc4388.utility.controller.IHandController;

public class RunExtenderOutIn extends CommandBase {
  private Intake m_intake;
  private boolean isOut = false;
  private long startTime;

  /**
   * Uses input from opperator triggers to control intake motor.
   * The right trigger will run the intake in and the left trigger will run it out.
   * @param subsystem pass the Intake subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public RunExtenderOutIn(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = subsystem;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isOut = !isOut;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isOut){
      m_intake.runExtender(0.3);
    } else {
      m_intake.runExtender(-0.3);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runExtender(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (startTime + 3000 < System.currentTimeMillis()) {
      return true;
    }
    return false;
  }
}
