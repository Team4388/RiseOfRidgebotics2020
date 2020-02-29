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

public class ShooterVelocityControlPID extends CommandBase {
  Shooter m_shooter;
  double m_targetVel;
  double m_actualVel;
  /**
   * Creates a new ShooterVelocityControlPID.
   * @param subsystem The Shooter subsytem
   * @param targetVel The target velocity
   */
  public ShooterVelocityControlPID(Shooter subsystem) {
    m_shooter = subsystem;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runDrumShooterVelocityPID(m_shooter.addFireVel(), m_shooter.m_shooterFalcon.getSelectedSensorVelocity());
    m_shooter.runAngleAdjustPID(m_shooter.addFireAngle());
    SmartDashboard.putNumber("Fire Velocity", m_shooter.addFireVel());
    SmartDashboard.putNumber("Fire Angle", m_shooter.addFireAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Tells wether the target velocity has been reached
    double upperBound = m_targetVel + 300;
    double lowerBound = m_targetVel - 300;
    if (m_actualVel < upperBound && m_actualVel > lowerBound){
      SmartDashboard.putBoolean("ShooterVelocityPID Finished", true);
      return true;
    }
    else{
      SmartDashboard.putBoolean("ShooterVelocityPID Finished", false);
      return false;
    }
  }
}
