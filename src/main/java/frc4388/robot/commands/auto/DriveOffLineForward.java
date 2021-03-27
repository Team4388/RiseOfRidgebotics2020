/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.RobotContainer;
import frc4388.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveOffLineForward extends SequentialCommandGroup {
  /**
   * Creates a new DriveOffLineForward.
   */
  public DriveOffLineForward(Drive drive, RobotContainer robotContainer, RamseteCommand[] paths) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    addCommands(
      new InstantCommand(() -> drive.switchReversed(true)),
      new InstantCommand(() -> robotContainer.resetOdometry(new Pose2d())),
      paths[0]
    );
  }
}
