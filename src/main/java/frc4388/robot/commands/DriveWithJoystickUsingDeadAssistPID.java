/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.utility.controller.IHandController;

public class DriveWithJoystickUsingDeadAssistPID extends CommandBase {
  Drive m_drive;
  double m_targetGyro, m_currentGyro;
  long m_lastTime, m_deltaTime;
  IHandController m_controller;

  /**
   * Creates a new DriveWithJoystickUsingDeadAssistPID to control the drivetrain with an Xbox controller.
   * Applies a curved ramp to the input from the controllers to make the robot less "touchy".
   * Also uses PIDs to keep the robot on course when given a "dead" or 0 input.
   * @param subsystem pass the Drive subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   * @param controller pass the Driver {@link frc4388.utility.controller.IHandController#getClass() IHandController} using the
   * {@link frc4388.robot.RobotContainer#getDriverJoystick() getDriverJoystick()} method in
   * {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public DriveWithJoystickUsingDeadAssistPID(Drive subsystem, IHandController controller) {
    // Use addRequirements() here to declare subsystem dependencies.    
    m_drive = subsystem;
    m_controller = controller;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lastTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    double moveInput = -m_controller.getLeftYAxis();
    double steerInput = m_controller.getRightXAxis();
    double moveOutput = 0;
    double steerOutput = 0;
    m_deltaTime = System.currentTimeMillis() - m_lastTime;
    m_lastTime = System.currentTimeMillis();

    /* If move stick is being used */
    if (moveInput != 0) {
      /* Curves the moveInput to be slightly more gradual at first */
      if (moveInput >= 0) {
        moveOutput = -Math.cos(1.571*moveInput)+1;
      } else {
        moveOutput = Math.cos(1.571*moveInput)-1;
      }

      /* If steer stick is being used. */
      if (steerInput != 0) {
        double cosMultiplier = .45;
        double deadzone = .2;
        /* Curves the steer output to be similarily gradual */
        if (steerInput > 0){
          steerOutput = -cosMultiplier*Math.cos(1.571*steerInput)+(cosMultiplier+deadzone);
        } else {
          steerOutput = cosMultiplier*Math.cos(1.571*steerInput)-(cosMultiplier+deadzone);
        }
        resetGyroTarget();
        m_drive.driveWithInput(moveOutput, steerOutput);
        System.out.println("Driving With Input");
      }
      /* If only the move stick is being used */
      else {
        updateGyroTarget(steerInput);
        m_drive.driveWithInputAux(moveOutput, m_targetGyro);
        System.out.println("Driving with Input Aux with Target: " + m_targetGyro);
      }
    }
    /* If the move stick is not being used */
    else {
      updateGyroTarget(steerInput);
      m_drive.runDriveStraightVelocityPID(0, m_targetGyro);
      System.out.println("Driving with Velocity PID with Target: " + m_targetGyro);
    }
  }

  /**
   * If AuxPID is enabled, then update using the steer input
   */
  private void updateGyroTarget(double steerInput) {
    m_targetGyro += 2 * steerInput * m_deltaTime;
  
    m_targetGyro = MathUtil.clamp(  m_targetGyro, 
                                    m_currentGyro-(DriveConstants.TICKS_PER_GYRO_REV/4),
                                    m_currentGyro+(DriveConstants.TICKS_PER_GYRO_REV/4));
  }

  /**
   * set target angle to current angle (prevents buildup of gyro error).
   */
  private void resetGyroTarget() {
    m_targetGyro = m_currentGyro;
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
