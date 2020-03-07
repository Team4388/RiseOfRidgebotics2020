/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.subsystems.Shooter;
import frc4388.utility.controller.XboxController;

public class TrimShooter extends CommandBase {
  private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);
  public double turretTrim = 0;
  public double hoodTrim = 0;

  public Shooter m_shooter;
  /**
   * Trims the shooter based on the D-Pad inputs
   * @param shootSub The Shooter subsytems
   */
  public TrimShooter(Shooter shootSub) {
    m_shooter = shootSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_operatorXbox.getDPadTop()){
      hoodTrim += 1;
    }
    else if(m_operatorXbox.getDPadBottom()){
      hoodTrim -= 1;
    }
    else if(m_operatorXbox.getDPadRight()){
      turretTrim += 1;
    }
    else if(m_operatorXbox.getDPadLeft()){
      turretTrim -= 1;
    }

    m_shooter.shooterTrims.m_turretTrim = turretTrim;
    m_shooter.shooterTrims.m_hoodTrim = hoodTrim;
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
