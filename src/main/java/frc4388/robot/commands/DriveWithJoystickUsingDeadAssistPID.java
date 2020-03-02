/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Pneumatics;
import frc4388.utility.controller.IHandController;

public class DriveWithJoystickUsingDeadAssistPID extends CommandBase {
  Drive m_drive;
  Pneumatics m_pneumatics;
  double m_targetGyro, m_currentGyro;
  double m_stopPos;
  long m_currTime, m_deltaTime;
  long m_deadTimeSteer, m_deadTimeMove;
  long m_deadTimeout = 100;
  IHandController m_controller;
  boolean m_isInterrupted;

  /* Deadassist Constants */
  final float stopPosVelCoefLow = 1;
  final float stopPosVelCoefHigh = 3;
  final float cosMultiplierLow = 0.55f;
  final float cosMultiplierHigh = 0.35f;
  final float targetAngleCoefLow = 5;
  final float targetAngleCoefHigh = 5;
  final float gyroVelCoefLow = 1;
  final float gyroVelCoefHigh = 3;

  /* Deadassist Coeficients */
  final float stopPosVelCoef = 1;
  final float cosMultiplier = 0.55f;
  final float targetAngleCoef = 5;
  final float gyroVelCoef = 1;

  /**
   * Creates a new DriveWithJoystickUsingDeadAssistPID to control the drivetrain with an Xbox controller.
   * Applies a curved ramp to the input from the controllers to make the robot less "touchy".
   * Also uses PIDs to keep the robot on course when given a "dead" or 0 input.
   * @param subsystemDrive pass the Drive subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   * @param controller pass the Driver {@link frc4388.utility.controller.IHandController#getClass() IHandController} using the
   * {@link frc4388.robot.RobotContainer#getDriverJoystick() getDriverJoystick()} method in
   * {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public DriveWithJoystickUsingDeadAssistPID(Drive subsystemDrive, Pneumatics subsystemPneumatics, IHandController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystemDrive;
    m_pneumatics = subsystemPneumatics;
    m_controller = controller;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currTime = System.currentTimeMillis();
    resetGyroTarget();
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

    if (m_isInterrupted) {
      resetGyroTarget();
      m_isInterrupted = false;
    }

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

    /* Curves the moveInput to be slightly more gradual at first */
    if (moveInput >= 0) {
      moveOutput = -Math.cos(1.571*moveInput)+1;
    } else {
      moveOutput = Math.cos(1.571*moveInput)-1;
    }

    if (m_pneumatics.m_isSpeedShiftHigh) {
      runDriveWithInput(moveOutput, steerInput);
      resetGyroTarget();
    }
    /* If move stick has been pressed within 1 sec */
    else if (m_currTime - m_deadTimeMove < m_deadTimeout) {
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

  private void runDriveWithInput(double move, double steerInput) {
    double cosMultiplier = .70;
    double steerOutput = 0;
    double deadzone = .1;
    /* Curves the steer output to be similarily gradual */
    if (steerInput > 0){
      steerOutput = -(cosMultiplier - deadzone)*Math.cos(1.571*steerInput)+(cosMultiplier);
    } else if (steerInput < 0) {
      steerOutput = (cosMultiplier - deadzone)*Math.cos(1.571*steerInput)-(cosMultiplier);
    }

    m_drive.driveWithInput(move, steerOutput);
    System.out.println("Driving With Input");
  }

  private void runDriveStraight(double move) {
    m_drive.driveWithInputAux(move * 3/4, m_targetGyro);
    System.out.println("Driving Straight with Target: " + m_targetGyro);
  }

  private void runStoppedTurn(double steer) {
    double cosMultiplier = 0.55;
    double steerOutput = 0;
    double deadzone = .2;
    /* Curves the steer output to be similarily gradual */
    if (steer > 0) {
      steerOutput = -cosMultiplier*Math.cos(1.571*steer)+(cosMultiplier+deadzone);
    } else if (steer < 0) {
      steerOutput = cosMultiplier*Math.cos(1.571*steer)-(cosMultiplier+deadzone);
    }

    updateGyroTarget(steerOutput);
    double currentPos = m_drive.m_rightFrontMotorPos;
    if (Math.abs(currentPos - m_stopPos) > 200) {
      m_drive.runDrivePositionPID(m_stopPos, m_targetGyro);
    } else {
      m_drive.driveWithInputAux(0, m_targetGyro);
    }
    System.out.println("Turning with Target: " + m_targetGyro);
  }

  /**
   * If AuxPID is enabled, then update using the steer input
   */
  private void updateGyroTarget(double steerInput) {
    m_targetGyro -= 5 * steerInput * m_deltaTime;
    m_targetGyro = MathUtil.clamp(  m_targetGyro,
                                    m_currentGyro-(DriveConstants.TICKS_PER_GYRO_REV/3),
                                    m_currentGyro+(DriveConstants.TICKS_PER_GYRO_REV/3));
  }

  /**
   * set target angle to current angle (prevents buildup of gyro error).
   */
  private void resetGyroTarget() {
    //m_targetGyro = m_currentGyro;
    m_targetGyro =  m_currentGyro
                    + m_drive.getTurnRate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_isInterrupted = interrupted;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
