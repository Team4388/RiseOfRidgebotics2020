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

public class StorageFirePID extends CommandBase {
  Storage m_storage;
  double m_intakeStartPos;
  
  /**
   * Runs the Storage until shoot beam is empty, then ends
   * @param storageSub The Storage subsytem
   */
  public StorageFirePID(Storage storageSub) {
    m_storage = storageSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeStartPos = m_storage.getEncoderPosInches();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_storage.runStoragePositionPID(m_intakeStartPos + StorageConstants.STORAGE_FULL_BALL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = (m_intakeStartPos + StorageConstants.STORAGE_FULL_BALL) - m_storage.getEncoderPosInches();
    if (m_storage.getEncoderVel() == 0 && Math.abs(error) < 0.5) {
      return true;
    }
    return false;
  }
}
