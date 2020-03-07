/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.subsystems.Storage;

public class StorageRun extends CommandBase {
  Storage m_storage;
  /**
   * Runs the Storage at a speed
   * @param storageSub The Storage subsytem
   */
  public StorageRun(Storage storageSub) {
    m_storage = storageSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_storage.runStorage(StorageConstants.STORAGE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storage.runStorage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
