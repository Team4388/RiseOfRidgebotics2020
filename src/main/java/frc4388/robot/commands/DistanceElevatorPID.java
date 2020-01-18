/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Elevator;

public class DistanceElevatorPID extends CommandBase {
  Elevator m_elevator;
  double m_distance;
  double m_target1;
  double m_target2;

  /**
   * Creates a new DistanceElevatorPID.
   */
  public DistanceElevatorPID(Elevator subsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = subsystem;
    m_distance = distance;
    addRequirements(m_elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_target1 = m_elevator.m_talon1.getActiveTrajectoryPosition() + m_distance;
    m_target2 = m_elevator.m_talon2.getActiveTrajectoryPosition() + m_distance;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.runElevatorPositionPID(m_elevator.m_talon1, m_target1);
    m_elevator.runElevatorPositionPID(m_elevator.m_talon2, m_target2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_elevator.m_talon1.getActiveTrajectoryPosition() - m_target1) <100 ) {
      return true;
    } else{
      return false;
    }
  }
}
