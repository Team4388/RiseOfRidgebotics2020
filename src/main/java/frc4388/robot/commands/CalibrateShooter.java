/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;

public class CalibrateShooter extends CommandBase {
  Shooter m_shooter;
  ShooterAim m_shooterAim;
  /**
   * Creates a new CalibrateShooter.
   */
  public CalibrateShooter(Shooter shootSub, ShooterAim aimSub) {
    m_shooter = shootSub;
    m_shooterAim = aimSub;
    addRequirements(m_shooter, m_shooterAim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.m_angleAdjustMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_shooter.m_angleAdjustMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_shooter.m_angleEncoder.setPosition(0);
    m_shooter.m_angleAdjustMotor.set(-0.1);

    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_shooterAim.m_shooterRotateEncoder.setPosition(0);
    m_shooterAim.m_shooterRotateMotor.set(0.075);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.m_angleAdjustMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_shooter.m_angleAdjustMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_shooterAim.m_shooterRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
