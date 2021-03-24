/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;

public class ShooterGoalPosition extends CommandBase {
  Shooter m_shooter;
  ShooterHood m_hood;
  ShooterAim m_aim;
  /**
   * Creates a new ShooterGoalPosition.
   */
  public ShooterGoalPosition(Shooter shooterSub, ShooterHood hoodSub, ShooterAim aimSub) {
    m_shooter = shooterSub;
    m_hood = hoodSub;
    m_aim = aimSub;
    addRequirements(m_shooter,m_hood,m_aim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runDrumShooterVelocityPID(5000);
    m_hood.runAngleAdjustPID(4);
    m_aim.runshooterRotatePID(-26.5);
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
