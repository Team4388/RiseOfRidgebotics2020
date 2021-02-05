/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.commands.intake.RunIntake;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveOffLineForward extends SequentialCommandGroup {
  /**
   * Creates a new DriveOffLineForward.
   */
  public DriveOffLineForward(Drive drive, RamseteCommand[] paths) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    Intake m_intake = new Intake();

    addCommands(
      paths[0],
      new RunIntake(m_intake)
    );
  }
}
