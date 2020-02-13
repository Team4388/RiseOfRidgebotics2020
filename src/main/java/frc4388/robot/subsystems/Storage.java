/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.StorageConstants;

public class Storage extends SubsystemBase {
  private CANSparkMax m_storageMotor = new CANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
  private DigitalInput[] m_beamSensors = new DigitalInput[6];

  CANPIDController m_storagePIDController = m_storageMotor.getPIDController();

  CANEncoder m_encoder = m_storageMotor.getEncoder();

  /**
   * Creates a new Storage.
   */
  public Storage() {
    resetEncoder();

    m_beamSensors[0] = new DigitalInput(StorageConstants.BEAM_SENSOR_DIO_0);
    m_beamSensors[1] = new DigitalInput(StorageConstants.BEAM_SENSOR_DIO_1);
    m_beamSensors[2] = new DigitalInput(StorageConstants.BEAM_SENSOR_DIO_2);
    m_beamSensors[3] = new DigitalInput(StorageConstants.BEAM_SENSOR_DIO_3);
    m_beamSensors[4] = new DigitalInput(StorageConstants.BEAM_SENSOR_DIO_4);
    m_beamSensors[5] = new DigitalInput(StorageConstants.BEAM_SENSOR_DIO_5);
  }

  @Override
  public void periodic() {
    // NO
  }

  /**
   * Runs storage motor
   * 
   * @param input the voltage to run motor at
   */
  public void runStorage(final double input) {
    m_storageMotor.set(input);
    final boolean beam_on = m_beamSensors[0].get();

    if (beam_on) {
      System.err.println("Beam on");
    } else {
      System.err.println("Beam off");
    }

  }

  public void resetEncoder()
  {
    m_encoder.setPosition(0);
  }

  /* Storage PID Control */
  public void runStoragePositionPID(double targetPos, double kP, double kI, double kD, double kIz, double kF, double kmaxOutput, double kminOutput)
  {
    // Set PID Coefficients
    m_storagePIDController.setP(kP);
    m_storagePIDController.setI(kI);
    m_storagePIDController.setD(kD);
    m_storagePIDController.setIZone(kIz);
    m_storagePIDController.setFF(kF);
    m_storagePIDController.setOutputRange(kminOutput, kmaxOutput);

    m_storagePIDController.setReference(targetPos, ControlType.kPosition);
  }

  public void getEncoderPos()
  {
    m_encoder.getPosition();
  }
}
