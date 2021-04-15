/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Leveler;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.utility.controller.IHandController;

public class RunHoodWithJoystick extends CommandBase {
  private ShooterHood m_hood;
  private IHandController m_controller;

  /**
   * Creates a new RunLevelerWithJoystick to control the leveler with an Xbox controller.
   * @param subsystem pass the Hood subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   * @param controller pass the Operator {@link frc4388.utility.controller.IHandController#getClass() IHandController} using the
   * {@link frc4388.robot.RobotContainer#getOperatorJoystick() getOperatorJoystick()} method in
   * {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public RunHoodWithJoystick(ShooterHood subsystem, IHandController controller) {
    m_hood = subsystem;
    m_controller = controller;
    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = m_controller.getRightYAxis();
    m_hood.runHood(input);
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
