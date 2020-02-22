/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StorageIntakeGroup extends SequentialCommandGroup {
  /**
   * Creates a new StorageIntakeGroup.
   */
  public StorageIntakeGroup(Intake m_intake, Storage m_storage) {
    addCommands(
      new StoragePrepIntake(m_intake, m_storage), 
      new StorageIntake(m_intake, m_storage), 
      new StorageIntakeFinal(m_storage)
      );
  }
}
