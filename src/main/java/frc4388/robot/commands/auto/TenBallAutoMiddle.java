/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.commands.shooter.CalibrateShooter;
import frc4388.robot.commands.shooter.PrepChecker;
import frc4388.robot.commands.shooter.ShootPrepGroup;
import frc4388.robot.commands.storage.RunStorage;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TenBallAutoMiddle extends SequentialCommandGroup {
  /**
   * Creates a new TenBallAutoMiddle.
   */
  public TenBallAutoMiddle(ShooterHood shooterHood, Storage storage, Intake intake, Shooter shooter, ShooterAim shooterAim, Drive drive, RamseteCommand[] paths) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new Wait(drive, 0.1, 0),
        new CalibrateShooter(shooter, shooterAim, shooterHood)
      ),
      new ParallelDeadlineGroup(
        new Wait(drive, 1, 0),
        new RunCommand(() -> shooterAim.runShooterWithInput(-0.75), shooterAim)
      ),
      new ParallelDeadlineGroup(
        new Wait(drive, 4, 0),
        new PrepChecker(shooter, shooterAim),
        new RunCommand(() -> intake.runExtender(IntakeConstants.EXTENDER_SPEED), intake),
        new ShootPrepGroup(shooter, shooterAim, shooterHood, storage)
      ),
      new ParallelDeadlineGroup(
        new ShootPrepGroup(shooter, shooterAim, shooterHood, storage),
        new RunStorage(storage)
      )
      //paths[0],
      //paths[1]
    );
  }
}
