/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GotoCoordinates extends SequentialCommandGroup {
  Drive m_drive;

  double m_xTarget;
  double m_yTarget;
  double m_currentAngle;
  double m_hypotDist;
  double m_endAngle;

  /**
   * Creates a new GotoPosition.
   */
  public GotoCoordinates(Drive subsystem, double xTarget, double yTarget, double endAngle) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    m_drive = subsystem;

    m_xTarget = xTarget;
    m_yTarget = yTarget;

    m_hypotDist = Math.sqrt((m_xTarget*m_xTarget) + (m_yTarget*m_yTarget));

    m_currentAngle = calcAngle();

    SmartDashboard.putNumber("Current Angle Coordinates", m_currentAngle); 

    m_endAngle = endAngle;


    addCommands(  new TurnDegrees(m_drive, m_currentAngle),
                  new Wait(m_drive, 0.1),
                  new DriveStraightToPositionPID(m_drive, m_hypotDist), 
                  new TurnDegrees(m_drive, m_endAngle - m_currentAngle));
  }

  public boolean isQuadrantThree() {
    if ((m_xTarget < 0) && (m_yTarget < 0)) {
      return true;
    } else {
      return false;
    }
  }

  public double calcAngle() {
    if (isQuadrantThree()) {
      return 360 + (Math.atan2(m_yTarget, m_xTarget) * (180 / Math.PI)) - 90;
    } else {
      return (Math.atan2(m_yTarget, m_xTarget) * (180 / Math.PI)) - 90;
    }
  }

}
