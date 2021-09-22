/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.commands.intake.RunExtenderOutIn;
import frc4388.robot.commands.intake.RunIntake;
import frc4388.robot.commands.shooter.CalibrateShooter;
import frc4388.robot.commands.shooter.TrackTarget;
import frc4388.robot.subsystems.Drive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FiveBallBottom extends SequentialCommandGroup {
  /**
   * Creates a new FiveBallBottom.
   */
  public FiveBallBottom(ShooterHood shooterHood, Storage storage, Intake intake, Shooter shooter, ShooterAim shooterAim, Drive drive, RamseteCommand[] paths) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //TODO Rewrite
    addCommands(
      //Shoot and Extend Intake
      new CalibrateShooter(shooter, shooterAim, shooterHood),
      new ParallelDeadlineGroup(
        new Wait(drive,5),
        new TrackTarget(shooterAim),
        new RunCommand(() -> shooterHood.runAngleAdjustPID(shooterHood.addFireAngle())),
        new RunExtenderOutIn(intake)
      ),
      //Intake and Path
      new ParallelDeadlineGroup(
        paths[0],
        new RunIntake(intake)
      ),
      //Shoot
      new ParallelDeadlineGroup(
        new Wait(drive,5),
        new TrackTarget(shooterAim),
        new RunCommand(() -> shooterHood.runAngleAdjustPID(shooterHood.addFireAngle()))
      )
    );
  }
}
