/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootPrepGroup extends ParallelCommandGroup {
  /**
   * Preps the Shooter to be fired
   */
  public ShootPrepGroup(Shooter m_shooter, ShooterAim m_shooterAim, Storage m_storage) {
    super(
      new TrackTarget(m_shooter, m_shooterAim),
      new ShooterVelocityControlPID(m_shooter, m_shooter.addFireVel()),
      new HoodAdjustPID(m_shooter),
      new StoragePrepAim(m_storage)
      );
  }
}