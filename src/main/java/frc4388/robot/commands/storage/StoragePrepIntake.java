/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Storage;

public class StoragePrepIntake extends CommandBase {
  public Intake m_intake;
  public Storage m_storage;
  public double startTime;

  /**
   * Prepares the Storage for intaking
   * @param inSub The Intake subsystem
   * @param storeSub the Storage subsystem
   */
  public StoragePrepIntake(Intake inSub, Storage storeSub) {
    m_intake = inSub;
    m_storage = storeSub;
    addRequirements(m_intake);
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
    if (m_storage.getBeam(StorageConstants.BEAM_SENSOR_STORAGE)){
      m_storage.runStorage(-StorageConstants.STORAGE_SPEED);
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
    if (!m_storage.getBeam(StorageConstants.BEAM_SENSOR_INTAKE) || startTime + StorageConstants.STORAGE_TIMEOUT <= System.currentTimeMillis()){
      return true;
    }
    return false;
  }
}