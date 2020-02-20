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
  double m_stopPos;
  long m_currTime, m_deltaTime;
  long m_deadTimeSteer, m_deadTimeMove;
  long m_deadTimeout = 100;
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
    m_currTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    double moveInput = -m_controller.getLeftYAxis();
    double steerInput = m_controller.getRightXAxis();
    double moveOutput = 0;
    m_deltaTime = System.currentTimeMillis() - m_currTime;
    m_currTime = System.currentTimeMillis();

    /* If move stick is being used */
    if (moveInput != 0) {
      m_deadTimeMove = m_currTime;
      m_stopPos = m_drive.m_rightFrontMotor.getSelectedSensorPosition()
                  + (m_drive.m_rightFrontMotor.getSelectedSensorVelocity());
    }
    /* If steer stick is being used */
    if (steerInput != 0) {
      m_deadTimeSteer = m_currTime;
    }

    /* If move stick has been pressed within 1 sec */
    if (m_currTime - m_deadTimeMove < m_deadTimeout) {
      /* Curves the moveInput to be slightly more gradual at first */
      if (moveInput >= 0) {
        moveOutput = -Math.cos(1.571*moveInput)+1;
      } else {
        moveOutput = Math.cos(1.571*moveInput)-1;
      }

      /* If steer stick has not been used for less than 1 sec */
      if (m_currTime - m_deadTimeSteer < m_deadTimeout) {
        runDriveWithInput(moveOutput, steerInput);
        resetGyroTarget();
      }
      /* If steer stick has not been used for 1 sec */
      else {
        runDriveStraight(moveOutput);
      }
    }
    /* If the move stick has not been used for 1 sec */
    else {
      runStoppedTurn(steerInput);
    }
  }

  private void runDriveWithInput(double move, double steer) {
    double cosMultiplier = .45;
    double steerOutput = 0;
    double deadzone = .2;
    /* Curves the steer output to be similarily gradual */
    if (steer > 0){
      steerOutput = -cosMultiplier*Math.cos(1.571*steer)+(cosMultiplier+deadzone);
    } else {
      steerOutput = cosMultiplier*Math.cos(1.571*steer)-(cosMultiplier+deadzone);
    }
    m_drive.driveWithInput(move, steerOutput);
    System.out.println("Driving With Input");
  }

  private void runDriveStraight(double move) {
    m_drive.driveWithInputAux(move * 3/4, m_targetGyro);
    System.out.println("Driving Straight with Target: " + m_targetGyro);
  }

  private void runStoppedTurn(double steer) {
    updateGyroTarget(steer);
    m_drive.runDrivePositionPID(m_stopPos, m_targetGyro);
    System.out.println("Turning with Target: " + m_targetGyro);
  }

  /**
   * If AuxPID is enabled, then update using the steer input
   */
  private void updateGyroTarget(double steerInput) {
    m_targetGyro -= 5 * steerInput * m_deltaTime;
    m_targetGyro = MathUtil.clamp(  m_targetGyro,
                                    m_currentGyro-(DriveConstants.TICKS_PER_GYRO_REV/8),
                                    m_currentGyro+(DriveConstants.TICKS_PER_GYRO_REV/8));
  }

  /**
   * set target angle to current angle (prevents buildup of gyro error).
   */
  private void resetGyroTarget() {
    m_targetGyro = m_currentGyro;
    m_targetGyro =  m_currentGyro
                    + m_drive.getTurnRate();
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
