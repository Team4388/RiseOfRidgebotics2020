/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.storage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.subsystems.Storage;

public class StoragePrepAim extends CommandBase {
  Storage m_storage;
  double startTime;
  /**
   * Prepares the Storage for aiming
   * @param storeSub The Storage subsystem
   */
  public StoragePrepAim(Storage storeSub) {
    m_storage = storeSub;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_storage.getBeam(1)){
      //m_storage.runStorage(StorageConstants.STORAGE_SPEED);
    }
    else{
      m_storage.runStorage(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (!m_storage.getBeam(1) || startTime + StorageConstants.STORAGE_TIMEOUT <= System.currentTimeMillis()){
      m_storage.m_isStorageReadyToFire = true;
      return true;
    } else {
      m_storage.m_isStorageReadyToFire = false;
      return false;
    }*/
    return true;
  }
}
