/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.subsystems.Intake;
import frc4388.utility.controller.IHandController;

public class RunExtenderOutIn extends CommandBase {
  private Intake m_intake;

  CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, MotorType.kBrushless);
  CANSparkMax m_extenderMotor = new CANSparkMax(IntakeConstants.EXTENDER_SPARK_ID, MotorType.kBrushless);
  CANDigitalInput m_extenderForwardLimit;
  CANDigitalInput m_extenderReverseLimit;

  

  /**
   * Uses input from opperator to run the extender motor.
   * The left bumper will run the extender in and out.
   * @param subsystem pass the Intake subsystem from {@link frc4388.robot.RobotContainer#RobotContainer() RobotContainer}
   */
  public RunExtenderOutIn(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = subsystem;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.isExtended = !m_intake.isExtended;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.isExtended){
      m_intake.runExtender(0.3);
    } else {
      m_intake.runExtender(-0.3);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runExtender(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intake.isExtended && m_extenderForwardLimit.get() == true){
      return true;
    }
    
    else if(m_intake.isExtended && m_extenderReverseLimit.get() == true){
      return true;
    }
    
    else{
      return false;
    }
  }
}
