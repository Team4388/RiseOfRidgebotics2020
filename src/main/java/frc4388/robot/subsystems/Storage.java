/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Gains;
import frc4388.robot.Constants.StorageConstants;

public class Storage extends SubsystemBase {
  public CANSparkMax m_storageMotor = new CANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
  private DigitalInput[] m_beamSensors = new DigitalInput[6];

  CANPIDController m_storagePIDController = m_storageMotor.getPIDController();

  CANEncoder m_encoder = m_storageMotor.getEncoder();

  Gains storageGains = StorageConstants.STORAGE_GAINS;

  Intake m_intake;


  /**
   * Creates a new Storage.
   */
  public Storage() {
    resetEncoder();
    m_beamSensors[1] = new DigitalInput(StorageConstants.BEAM_SENSOR_SHOOTER);
    m_beamSensors[2] = new DigitalInput(StorageConstants.BEAM_SENSOR_USELESS);
    m_beamSensors[3] = new DigitalInput(StorageConstants.BEAM_SENSOR_STORAGE);
    m_beamSensors[4] = new DigitalInput(StorageConstants.BEAM_SENSOR_INTAKE);
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
  
  public void runStorage(double input) {
    m_storageMotor.set(input);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0);
  }

  public void testBeams(){
    SmartDashboard.putBoolean("Beam 0", m_beamSensors[0].get());
    SmartDashboard.putBoolean("Beam 1", m_beamSensors[1].get());
  }

  /* Storage PID Control */
  public void runStoragePositionPID(double targetPos){
    // Set PID Coefficients
    m_storagePIDController.setP(storageGains.m_kP);
    m_storagePIDController.setI(storageGains.m_kI);
    m_storagePIDController.setD(storageGains.m_kD);
    m_storagePIDController.setIZone(storageGains.m_kIzone);
    m_storagePIDController.setFF(storageGains.m_kF);
    m_storagePIDController.setOutputRange(StorageConstants.STORAGE_MIN_OUTPUT, storageGains.m_kmaxOutput);

    SmartDashboard.putNumber("Storage Position PID Target", targetPos);
    SmartDashboard.putNumber("Storage Position Pos", getEncoderPos());
    m_storagePIDController.setReference(targetPos, ControlType.kPosition);
  }


  public double getEncoderPos(){
    return m_encoder.getPosition();
  }

  public boolean getBeam(int id){
    return m_beamSensors[id].get();
  }

  public void setStoragePID(double position){
    m_storagePIDController.setReference(position , ControlType.kPosition);
  }
}
