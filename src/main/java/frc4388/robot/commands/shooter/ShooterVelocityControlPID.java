/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterHood;

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
    m_shooter.runDrumShooterVelocityPID(/*m_shooter.addFireVel()*/13000);
    //m_shooterHood.runAngleAdjustPID(m_shooterHood.addFireAngle());
    //SmartDashboard.putNumber("Fire Velocity", m_shooter.addFireVel());
    //SmartDashboard.putNumber("Fire Angle", m_shooter.addFireAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Tells whether the target velocity has been reached
    double upperBound = m_targetVel + ShooterConstants.DRUM_VELOCITY_BOUND;
    double lowerBound = m_targetVel - ShooterConstants.DRUM_VELOCITY_BOUND;
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
