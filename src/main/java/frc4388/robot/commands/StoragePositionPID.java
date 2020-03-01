/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.subsystems.Storage;

public class StoragePositionPID extends CommandBase {
  public Storage m_storage;
  double startPos;
  /**
   * Moves the storage a number of rotations
   * @param subsystem The Storage subsystem
   */
  public StoragePositionPID(Storage subsystem) {
    m_storage = subsystem;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_storage.runStoragePositionPID(StorageConstants.STORAGE_FULL_BALL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (startPos + StorageConstants.STORAGE_FULL_BALL == m_storage.getEncoderPos())
    {
      return true;
    }*/
    return false;
  }
}
