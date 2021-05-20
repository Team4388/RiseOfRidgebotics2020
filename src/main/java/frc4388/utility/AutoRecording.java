/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;

public class AutoRecording extends CommandBase {
  /**
   * Creates a new AutoRecording.
   */
  Drive m_drive;

  Pose2d start = new Pose2d();
  Pose2d prev = new Pose2d();
  Pose2d curr = new Pose2d();

  boolean isFirst;

  double currentVel = 0;
  double currentAcc = 0;

  double maxVel = Double.MIN_VALUE;
  double maxAcc = Double.MIN_VALUE;

  double EPSILON = 1e-5;

  int ctr = 0;

  public AutoRecording(Drive subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = m_drive.getPose();
    isFirst = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    prev = isFirst ? start : curr;
    isFirst = false;

    curr = m_drive.getPose();

    Transform2d diff = new Transform2d(prev, curr);

    if ((Math.abs(diff.getTranslation().getX()) > EPSILON) || (Math.abs(diff.getTranslation().getY()) > EPSILON)) {
      
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
