/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.subsystems.Drive;

public class TurnDegrees extends CommandBase {

  double m_targetAngle;
  Drive m_drive;
  double m_currentYawInTicks;
  double m_targetAngleTicksIn;
  double m_targetAngleTicksOut;
  int i;

  /**
   * Creates a new TurnDeg.
   */
  public TurnDegrees(double targetAngle, Drive subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_targetAngle = targetAngle;
    m_drive = subsystem;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetAngleTicksIn = (m_targetAngle / 360) * DriveConstants.TICKS_PER_GYRO_REV;
    m_currentYawInTicks = (m_drive.getGyroYaw() / 360) * DriveConstants.TICKS_PER_GYRO_REV;
    m_targetAngleTicksOut = m_targetAngleTicksIn + m_currentYawInTicks;

    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentYawInTicks = (m_drive.getGyroYaw() / 360) * DriveConstants.TICKS_PER_GYRO_REV;

    m_drive.runTurningPID(m_targetAngleTicksOut);

    SmartDashboard.putNumber("Turning Error", Math.abs(m_currentYawInTicks - m_targetAngleTicksOut));
    SmartDashboard.putNumber("Turning Target", m_targetAngleTicksOut);

    i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(m_drive.getTurnRate()) < 1) && (i > 5)) {
      return true;
    }
    return false;
  }
}

