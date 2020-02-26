/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Storage;

public class storageIntake extends CommandBase {
  public Intake m_intake;
  public Storage m_storage;
  /**
   * Creates a new storageIntake.
   */
  public storageIntake(Intake inSub, Storage storeSub) {
    m_intake = inSub;
    m_storage = storeSub;
    addRequirements(m_intake);
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_storage.getBeam(0)){
      m_storage.setStoragePID(m_storage.getEncoderPos() + 2);
      m_intake.runExtender(-0.3); 
    }
    else{
      m_intake.runExtender(0.3); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_storage.getBeam(0)){
      return true;
    }
    return false;
  }
}
