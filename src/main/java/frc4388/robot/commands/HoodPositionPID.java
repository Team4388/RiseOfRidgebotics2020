/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Shooter;

public class HoodPositionPID extends CommandBase {
  Shooter m_shooter;
  /**
   * Creates a new HoodPositionPID.
   */
  public HoodPositionPID(Shooter subSystem) {
    m_shooter = subSystem;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double firingAngle = (-0.47*m_shooter.addFireAngle())+40.5;
    SmartDashboard.putNumber("Shoot Angle From Equation", m_shooter.addFireAngle());
    SmartDashboard.putNumber("Fire Angle", firingAngle);
    m_shooter.runAngleAdjustPID(firingAngle);
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
