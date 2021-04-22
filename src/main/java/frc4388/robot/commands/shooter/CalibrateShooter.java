/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;

public class CalibrateShooter extends CommandBase {
  Shooter m_shooter;
  ShooterAim m_shooterAim;
  ShooterHood m_shooterHood;

  /**
   * Calibrates the turret by moving the hood all the way down and moving the turret all the way right, then reseting the encoders
   * @param shootSub The Shooter subsystem
   * @param aimSub The ShooterAim subsystem
   */
  public CalibrateShooter(Shooter shootSub, ShooterAim aimSub, ShooterHood hoodSub) {
    m_shooter = shootSub;
    m_shooterAim = aimSub;
    m_shooterHood = hoodSub;
    addRequirements(m_shooter, m_shooterHood, m_shooterAim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterHood.m_angleEncoder.setPosition(0);
    m_shooterHood.m_angleAdjustMotor.set(-ShooterConstants.HOOD_CALIBRATE_SPEED);

    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_shooterAim.m_shooterRotateEncoder.setPosition(0);
    m_shooterAim.m_shooterRotateMotor.set(ShooterConstants.TURRET_CALIBRATE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterAim.m_shooterRotateMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get() &&
      m_shooterHood.m_hoodDownLimit.get()) {
      return true;
    }
    return false;
  }
}
