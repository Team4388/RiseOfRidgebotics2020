/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootFullGroup extends SequentialCommandGroup {
  /**
   * Preps and Fires the Shooter
   * @param m_shooter The Shooter subsytem
   * @param m_shooterAim The ShooterAim subsystem
   * @param m_storage The Storage subsytem
   */
  public ShootFullGroup(Shooter m_shooter, ShooterAim m_shooterAim, Storage m_storage) {
    addCommands(
      new ShootPrepGroup(m_shooter, m_shooterAim, m_storage), 
      new ShootFireGroup(m_shooter, m_shooterAim, m_storage)
    );
  }
}
