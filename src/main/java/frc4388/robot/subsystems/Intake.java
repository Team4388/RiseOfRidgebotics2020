/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, MotorType.kBrushless);
  
  /**
   * Creates a new Intake.
   */
  public Intake() {
    
    m_intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs intake motor
   * @param input the percent output to run motor at
   */
  public void runIntake(double input) {
    m_intakeMotor.set(input);
  }
}
