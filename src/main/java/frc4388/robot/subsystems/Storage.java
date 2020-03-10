/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.StorageConstants;
import frc4388.utility.Gains;

public class Storage extends SubsystemBase {
  public CANSparkMax m_storageMotor = new CANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
  private DigitalInput m_beamShooter = new DigitalInput(StorageConstants.BEAM_SENSOR_SHOOTER);
  private DigitalInput m_beamUseless = new DigitalInput(StorageConstants.BEAM_SENSOR_USELESS);
  private DigitalInput m_beamStorage = new DigitalInput(StorageConstants.BEAM_SENSOR_STORAGE);
  private DigitalInput m_beamIntake = new DigitalInput(StorageConstants.BEAM_SENSOR_INTAKE);

  CANPIDController m_storagePIDController = m_storageMotor.getPIDController();

  CANEncoder m_encoder = m_storageMotor.getEncoder();

  Gains storageGains = StorageConstants.STORAGE_GAINS;

  Intake m_intake;

  public boolean m_isStorageReadyToFire = false;

  /**
   * Creates a new Storage.
   */
  public Storage() {
    resetEncoder();

    // Set PID Coefficients
    m_storagePIDController.setP(storageGains.m_kP);
    m_storagePIDController.setI(storageGains.m_kI);
    m_storagePIDController.setD(storageGains.m_kD);
    m_storagePIDController.setIZone(storageGains.m_kIzone);
    m_storagePIDController.setFF(storageGains.m_kF);
    m_storagePIDController.setOutputRange(storageGains.m_kminOutput, storageGains.m_kmaxOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Beam", getBeamIntake());
    SmartDashboard.putBoolean("Storage Beam", getBeamStorage());
    SmartDashboard.putBoolean("Upper Beam", getBeamUseless());
    SmartDashboard.putBoolean("Shooter Beam", getBeamShooter());
  }

  /**
   * Runs storage motor
   * 
   * @param input the percent output to run motor at
   */
  public void runStorage(double input) {
    m_storageMotor.set(input);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0);
  }

  /**
   * Runs Storage to a particular position
   * @param targetPos in inches
   */
  public void runStoragePositionPID(double targetPos){
    //SmartDashboard.putNumber("Storage Position PID Target", targetPos);
    //SmartDashboard.putNumber("Storage Position Pos", getEncoderPos());
    targetPos = InchesToMotorRots(targetPos);
    m_storagePIDController.setReference(targetPos, ControlType.kPosition);
  }

  /**
   * Runs Storage to a particular position
   * @param position in motor rotations
   */
  public void setStoragePID(double position){
    m_storagePIDController.setReference(position, ControlType.kPosition);
  }

  public double getEncoderPos(){
    return m_encoder.getPosition();
  }

  public double getEncoderPosInches(){
    return motorRotsToInches(getEncoderPos());
  }

  public double getEncoderVel(){
    return m_encoder.getVelocity();
  }

  /**
   * @param motorRots
   * @return inches
   */
  public double motorRotsToInches(double motorRots) {
    return motorRots * (1/StorageConstants.MOTOR_ROTS_PER_STORAGE_ROT) * (StorageConstants.INCHES_PER_STORAGE_ROT);
  }

  /**
   * @param inches
   * @return motorRots
   */
  public double InchesToMotorRots(double inches) {
    return inches * (1/StorageConstants.INCHES_PER_STORAGE_ROT) * (StorageConstants.MOTOR_ROTS_PER_STORAGE_ROT);
  }

  public boolean getBeamShooter(){
    return m_beamShooter.get();
  }

  public boolean getBeamUseless(){
    return m_beamUseless.get();
  }

  public boolean getBeamStorage(){
    return m_beamStorage.get();
  }

  public boolean getBeamIntake(){
    return m_beamIntake.get();
  }
}
