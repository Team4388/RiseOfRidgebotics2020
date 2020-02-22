/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Storage;

public class StoragePrepAim extends CommandBase {
  Storage m_storage;
  /**
   * Creates a new storagePrepAim.
   */
  public StoragePrepAim(Storage storeSub) {
    m_storage = storeSub;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_storage.getBeam(2) == false){
      m_storage.runStorage(0.5);
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
    if (m_storage.getBeam(2)){
      return true;
    }
    return false;
  }
}
