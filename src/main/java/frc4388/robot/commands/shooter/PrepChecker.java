/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.robot.subsystems.Storage;

public class PrepChecker extends CommandBase {
  Shooter m_shooter;
  ShooterAim m_shooterAim;
  ShooterHood m_shooterHood;
  Storage m_storage;

  boolean m_isDrumReady = false;
  boolean m_isAimReady = false;
  boolean m_isHoodReady = false;
  boolean m_isStorageReady = false;
  
  /**
   * Creates a new PrepChecker.
   * @param shooter used to read all shooter subsystems. Not used as a requirement so don't expect it to interrupt other commands.
   * @param storage reads storage in a similar way to shooter. Not used as a requirement.
   */
  public PrepChecker(Shooter shooter, Storage storage) {
    m_shooter = shooter;
    m_shooterAim = m_shooter.m_shooterAimSubsystem;
    m_shooterHood = m_shooter.m_shooterHoodSubsystem;
    m_storage = storage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isDrumReady = false;
    m_isAimReady = false;
    m_isHoodReady = false;
    m_isStorageReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isDrumReady = m_shooter.m_isDrumReady;              SmartDashboard.putBoolean("ShooterVelocityPID Finished", m_isDrumReady);
    m_isAimReady = m_shooterAim.m_isAimReady;             SmartDashboard.putBoolean("TrackTarget Finished", m_isAimReady);
    m_isHoodReady = m_shooterHood.m_isHoodReady;          SmartDashboard.putBoolean("HoodPosition Finished", m_isHoodReady);
    m_isStorageReady = m_storage.m_isStorageReadyToFire;  SmartDashboard.putBoolean("StoragePrepAim Finished", m_isStorageReady);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.m_isDrumReady = false;
    m_shooterAim.m_isAimReady = false;
    m_shooterHood.m_isHoodReady = false;
    m_storage.m_isStorageReadyToFire = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_isDrumReady && m_isAimReady && m_isHoodReady && m_isStorageReady) {
      return true;
    }
    return false;
  }
}
