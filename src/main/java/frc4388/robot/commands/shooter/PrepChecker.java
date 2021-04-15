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

public class PrepChecker extends CommandBase {
  Shooter m_shooter;
  ShooterAim m_shooterAim;

  /**
   * Creates a new PrepChecker.
   */
  public PrepChecker(Shooter shooter, ShooterAim shooterAim) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_shooterAim = shooterAim;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_shooterAim.m_isAimReady && m_shooter.m_isDrumReady) {
      return true;
    }

    return false;
  }
}
