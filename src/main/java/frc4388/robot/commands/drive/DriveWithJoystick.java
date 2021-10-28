/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants;
import frc4388.robot.Robot;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.Constants.MathConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Pneumatics;
import frc4388.utility.controller.IHandController;

public class DriveWithJoystick extends CommandBase {
  private Drive m_drive;
  private IHandController m_controller;
  private Pneumatics m_pneumatics;

  /**
   * Creates a new DriveWithJoystick to control the drivetrain with an Xbox controller.
   * Applies a curved ramp to the input from the controllers to make the robot less "touchy".
   * @param subsystem pass the Drive subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   * @param controller pass the Driver {@link frc4388.utility.controller.IHandController#getClass() IHandController} using the
   * {@link frc4388.robot.RobotContainer#getDriverJoystick() getDriverJoystick()} method in
   * {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public DriveWithJoystick(Drive subsystem, Pneumatics subsystem2, IHandController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_pneumatics = subsystem2;
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
    double moveInput = m_controller.getLeftYAxis() * DriveConstants.DRIVE_WITH_JOYSTICK_FACTOR;
    double steerInput = m_controller.getRightXAxis() * DriveConstants.DRIVE_WITH_JOYSTICK_FACTOR;

    double moveOutput = (moveInput >= 0 ? -1 : 1) * (Math.cos(MathConstants.PI_2 * moveInput) - 1);
    double cosMultiplier = m_pneumatics.m_isSpeedShiftHigh ? DriveConstants.COS_MULTIPLIER_HIGH : DriveConstants.COS_MULTIPLIER_LOW;
    double deadzone = 0.1;
    double steerOutput = (steerInput == 0 ? 0 : (steerInput > 0 ? -1 : 1)) * (Math.cos(MathConstants.PI_2 * steerInput) * (cosMultiplier - deadzone) + cosMultiplier);

    /*
    double outputLimit = 0.8;

    boolean isMoveOutputLimited = false;
    boolean isSteerOutputLimited = false;
    
    if (m_pneumatics.m_isSpeedShiftHigh) {
      if (isMoveOutputLimited) {
        if (moveOutput > outputLimit) {
          moveOutput = outputLimit;
        } else if(moveOutput < -outputLimit) {
          moveOutput = -outputLimit;
        }
      }

      if (isSteerOutputLimited) {
        if (steerOutput > outputLimit) {
          steerOutput = outputLimit;
        } else if(steerOutput < -outputLimit) {
          steerOutput = -outputLimit;
        }
      }
    } 
  */
    
    m_drive.driveWithInput(moveOutput, steerOutput * .8);
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
