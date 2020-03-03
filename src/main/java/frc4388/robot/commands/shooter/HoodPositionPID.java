/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.ShooterHood;

public class HoodPositionPID extends CommandBase {
  ShooterHood m_shooterHood;
  double firingAngle;

  /**
   * Creates a new HoodPositionPID.
   */
  public HoodPositionPID(ShooterHood subSystem) {
    m_shooterHood = subSystem;
    addRequirements(m_shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double slope = ShooterConstants.HOOD_CONVERT_SLOPE;
    double b = ShooterConstants.HOOD_CONVERT_B;
    firingAngle = (-slope*m_shooter.addFireAngle())+b;*/
    //SmartDashboard.putNumber("Shoot Angle From Equation", m_shooter.addFireAngle());
    //SmartDashboard.putNumber("Fire Angle", firingAngle);
    m_shooterHood.runAngleAdjustPID(firingAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double encoderPos = m_shooterHood.m_angleAdjustMotor.getEncoder().getPosition();
    if(encoderPos < firingAngle + 1 || encoderPos < firingAngle - 1){
      m_shooterHood.m_isHoodReady = true;
    } else {
      m_shooterHood.m_isHoodReady = false;
    }
    return false;
  }
}
