/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;
import frc4388.utility.controller.IHandController;

public class DriveWithJoystick extends CommandBase {
  private Drive m_drive;
  private IHandController m_controller;

  /**
   * Creates a new DriveWithJoystick to control the drivetrain with an Xbox controller.
   * Applies a curved ramp to the input from the controllers to make the robot less "touchy".
   * @param subsystem pass the Drive subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   * @param controller pass the Driver {@link frc4388.utility.controller.IHandController#getClass() IHandController} using the
   * {@link frc4388.robot.RobotContainer#getDriverJoystick() getDriverJoystick()} method in
   * {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public DriveWithJoystick(Drive subsystem, IHandController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_controller = controller;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveInput = -m_controller.getLeftYAxis();
    double steerInput = m_controller.getRightXAxis();
    double moveOutput = 0;
    double steerOutput = 0;
    if (moveInput >= 0){
      moveOutput = -Math.cos(1.571*moveInput)+1;
    } else {
      moveOutput = Math.cos(1.571*moveInput)-1;
    }

    double cosMultiplier = .55;
    double deadzone = .2;
    if (steerInput > 0){
      steerOutput = -cosMultiplier*Math.cos(1.571*steerInput)+(cosMultiplier+deadzone);
    } else if (steerInput < 0) {
      steerOutput = cosMultiplier*Math.cos(1.571*steerInput)-(cosMultiplier+deadzone);
    } else {
      steerOutput = 0;
    }

    m_drive.driveWithInput(moveOutput, steerOutput);
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
