/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc4388.robot.commands.storage.StorageFire;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootFireGroup extends ParallelDeadlineGroup {
  /**
   * Fires the shooter
   * @param m_shooter The Shooter subsytem
   * @param m_shooterAim The ShooterAim subsystem
   * @param m_storage The Storage subsytem
   */
  public ShootFireGroup(Shooter m_shooter, ShooterAim m_shooterAim, ShooterHood m_shooterHood, Storage m_storage) {
    super(
      new StorageFire(m_storage),
      new TrackTarget(m_shooterAim),
      new ShooterVelocityControlPID(m_shooter),
      new HoodPositionPID(m_shooterHood)
    );
  }
}