/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.subsystems.Shooter;

public class ShooterVelocityControlPID extends CommandBase {
  Shooter m_shooter;
  double m_targetVel;
  double m_actualVel;
  
  /**
   * Runs the drum at a velocity
   * @param subsystem The Shooter subsytem
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
    //Tells whether the target velocity has been reached
    m_actualVel = m_shooter.m_shooterFalconLeft.getSelectedSensorPosition();
    m_targetVel = m_shooter.addFireVel();
    double error = m_actualVel - m_targetVel;
    if (Math.abs(error) < ShooterConstants.DRUM_VELOCITY_BOUND){
      m_shooter.m_isDrumReady = true;
      m_shooter.runDrumShooterVelocityPID(m_targetVel);
    }
    else{
      m_shooter.m_isDrumReady = false;
      m_shooter.runDrumShooterVelocityPID(m_targetVel);
    }
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
