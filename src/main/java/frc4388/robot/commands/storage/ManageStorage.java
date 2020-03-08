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

public class ManageStorage extends CommandBase {
  Storage m_storage;

  /* Keeps track of which beam breaks are pressed */
  boolean isBallInIntake = false;
  boolean isBallInStorage = false;
  boolean isBallInShooter = false;

  /* Used for intaking a ball. Keeps track off when the 2nd ball in storage has moved */
  boolean m_isStorageEmpty = true;

  enum StorageMode{IDLE, INTAKE, RESET};
  StorageMode m_storageMode = StorageMode.IDLE;
  
  /**
   * Creates a new ManageStorage.
   */
  public ManageStorage(Storage m_robotStorage, StorageMode storageMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_storage = m_robotStorage;
    m_storageMode = storageMode;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBallInIntake = !m_storage.getBeam(StorageConstants.BEAM_SENSOR_INTAKE);
    isBallInStorage = !m_storage.getBeam(StorageConstants.BEAM_SENSOR_STORAGE);
    isBallInShooter = !m_storage.getBeam(StorageConstants.BEAM_SENSOR_SHOOTER);

    m_isStorageEmpty = !isBallInStorage;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isBallInIntake = !m_storage.getBeam(StorageConstants.BEAM_SENSOR_INTAKE);
    isBallInStorage = !m_storage.getBeam(StorageConstants.BEAM_SENSOR_STORAGE);
    isBallInShooter = !m_storage.getBeam(StorageConstants.BEAM_SENSOR_SHOOTER);

    SmartDashboard.putBoolean("Ball in Intake", isBallInIntake);
    SmartDashboard.putBoolean("Ball in Storage", isBallInStorage);
    SmartDashboard.putBoolean("Ball in Shooter", isBallInShooter);

    if (m_storageMode == StorageMode.IDLE) {
      runIdle();
    } else if (m_storageMode == StorageMode.INTAKE) {
      runIntake();
    } else if (m_storageMode == StorageMode.RESET) {
      runReset();
    }
  }

  /**
   * Intakes a ball.
   * Runs until the storage ball has moved past the
   * storage sensor and the intake ball has taken its place.
   */
  private void runIntake() {
    m_storage.runStorage(StorageConstants.STORAGE_SPEED);

    if (!m_isStorageEmpty && !isBallInStorage) {
      m_isStorageEmpty = true;
    }
    if (m_isStorageEmpty && isBallInStorage) {
      m_isStorageEmpty = false;
      m_storageMode = StorageMode.IDLE;
    }
  }

  /**
   * Idles until a ball is in the intake.
   * Also updates the status of the storage position
   */
  private void runIdle() {
    m_storage.runStorage(0);

    if (isBallInIntake) {
      m_storageMode = StorageMode.INTAKE;
    }
    m_isStorageEmpty = !isBallInStorage;
  }

  /**
   * Runs Storage out until the Intake Sensor is tripped.
   * Then switches into intake mode. This effectively
   * resets the position of the balls back to the bottom of the storage.
   */
  private void runReset() {
    m_storage.runStorage(-StorageConstants.STORAGE_SPEED);

    if (isBallInIntake) {
      m_storageMode = StorageMode.INTAKE;
    }
    m_isStorageEmpty = !isBallInStorage;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
