/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.utility.controller.IHandController;

public class DriveWithJoystickAuxPID extends CommandBase {
  Drive m_drive;
  double m_targetGyro;
  long lastTime;
  IHandController m_controller;

  /**
   * Creates a new DriveWithJoystickAuxPID.
   */
  public DriveWithJoystickAuxPID(Drive subsystem, IHandController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_controller = controller;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    lastTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentGyro = m_drive.m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN);
    double moveInput = m_controller.getLeftYAxis();
    double steerInput = m_controller.getRightXAxis();
    double moveOutput = 0;
    long deltaTime = System.currentTimeMillis() - lastTime;
    lastTime = System.currentTimeMillis();
    if (moveInput >= 0){
      moveOutput = -Math.cos(1.571*moveInput)+1;
    } else {
      moveOutput = Math.cos(1.571*moveInput)-1;
    }

    m_targetGyro += 2 * steerInput * deltaTime;

    m_targetGyro = MathUtil.clamp(m_targetGyro, 
                                  currentGyro-(DriveConstants.TICKS_PER_GYRO_REV/4),
                                  currentGyro+(DriveConstants.TICKS_PER_GYRO_REV/4));

    m_drive.driveWithInputAux(moveOutput, m_targetGyro);

    System.err.println("Target: " + m_targetGyro);
    System.err.println("Current: " + currentGyro);
    System.err.println();
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
