/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.commands.TrackTarget;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Storage;

public class ShootShooter extends CommandBase {
  Shooter m_shooter;
  Storage m_storage;
  private long startTime;
  private int ballNum;
  /**
   * Creates a new ShootAlll.
   */
  public ShootShooter(Shooter shootSub, Storage storeSub, int numBall) {
    ballNum = numBall;
    m_shooter = shootSub;
    m_storage = storeSub;
    addRequirements(m_shooter);
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    //new InstantCommand(() -> m_storage.storageAiming());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrackTarget track = new TrackTarget(m_shooter); //Used to pull velocity and angle from TrackTarget.java

      //Aiming
    if (startTime + 3000 > System.currentTimeMillis())
    {
      new ShooterVelocityControlPID(m_shooter, track.addFireVel());
    }
    
    else if(3000 + startTime + (1000 * ballNum) > System.currentTimeMillis()){
      new RunCommand(() -> m_storage.runStorage(0.5));
      new ShooterVelocityControlPID(m_shooter, track.addFireVel());
      new RunCommand(() -> m_shooter.runAngleAdjustPID(track.addFireAngle()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (3000 + startTime + (1000 * ballNum) > System.currentTimeMillis()){
      return true;
    }
    return false;
  }
}
