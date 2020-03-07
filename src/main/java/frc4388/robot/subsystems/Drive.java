/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.DriveConstants;
import frc4388.utility.Gains;

public class Drive extends SubsystemBase {
  /* Create Motors, Gyros, etc */
  public WPI_TalonFX m_leftFrontMotor = new WPI_TalonFX(DriveConstants.DRIVE_LEFT_FRONT_CAN_ID);
  public WPI_TalonFX m_rightFrontMotor = new WPI_TalonFX(DriveConstants.DRIVE_RIGHT_FRONT_CAN_ID);
  public WPI_TalonFX m_leftBackMotor = new WPI_TalonFX(DriveConstants.DRIVE_LEFT_BACK_CAN_ID);
  public WPI_TalonFX m_rightBackMotor = new WPI_TalonFX(DriveConstants.DRIVE_RIGHT_BACK_CAN_ID);
  public static PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.PIGEON_ID);
  public static GyroBase m_pigeonGyro;

  /* Drive objects to manage Drive Train */
  public DifferentialDrive m_driveTrain;
  public final DifferentialDriveOdometry m_odometry;
  public Orchestra m_orchestra;

  /* Pneumatics Subsystem */
  public Pneumatics m_pneumaticsSubsystem;

  /* Low Gear Gains */
  public static Gains m_gainsDistanceLow = DriveConstants.DRIVE_DISTANCE_GAINS_LOW;
  public static Gains m_gainsVelocityLow = DriveConstants.DRIVE_VELOCITY_GAINS_LOW;
  public static Gains m_gainsTurningLow = DriveConstants.DRIVE_TURNING_GAINS_LOW;
  public static Gains m_gainsMotionMagicLow = DriveConstants.DRIVE_MOTION_MAGIC_GAINS_LOW;

  /* High Gear Gains */
  public static Gains m_gainsDistanceHigh = DriveConstants.DRIVE_DISTANCE_GAINS_HIGH;
  public static Gains m_gainsVelocityHigh = DriveConstants.DRIVE_VELOCITY_GAINS_HIGH;
  public static Gains m_gainsTurningHigh = DriveConstants.DRIVE_TURNING_GAINS_HIGH;
  public static Gains m_gainsMotionMagicHigh = DriveConstants.DRIVE_MOTION_MAGIC_GAINS_HIGH;

  /* Timey Whimey */
  public long m_currentTimeMs = System.currentTimeMillis();
  public long m_lastTimeMs = m_currentTimeMs;
  public long m_deltaTimeMs = 0;
  public long m_currentTimeSec = m_currentTimeMs / 1000;

  /* Position Tracking */
  public double m_rightFrontMotorPos = 0;
  public double m_rightFrontMotorVel = 0;

  public double m_totalLeftDistanceInches = 0;
  public double m_totalRightDistanceInches = 0;

  public double m_currentLeftPosTicks = 0;
  public double m_currentRightPosTicks = 0;
  public double m_lastLeftPosTicks = 0;
  public double m_lastRightPosTicks = 0;

  public double m_lastAngleYaw = 0;
  public double m_currentAngleYaw = 0;

  public double m_lastAngleGotoCoordinates;
  /* Smart Dashboard Objects */
  SendableChooser<String> m_songChooser = new SendableChooser<String>();

  /* Misc */
  String m_currentSong = "";

  /**
   * Add your docs here.
   */
  public Drive() {
    /* factory default values */
    m_leftFrontMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_leftBackMotor.configFactoryDefault();
    m_rightBackMotor.configFactoryDefault();
    m_pigeon.configFactoryDefault();
    resetGyroYaw();

    m_pigeonGyro = getGyroInterface();

    /* Config Open Loop Ramp so we don't make sudden output changes */
    m_rightFrontMotor.configOpenloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.configOpenloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configOpenloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.configOpenloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);

    m_rightFrontMotor.configClosedloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.configClosedloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configClosedloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.configClosedloopRamp(DriveConstants.OPEN_LOOP_RAMP_RATE, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Config Supply Current Limit (Use only for debugging) */
    // m_rightFrontMotor.configSupplyCurrentLimit(DriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG);
    // m_leftFrontMotor.configSupplyCurrentLimit(DriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG);
    // m_rightBackMotor.configSupplyCurrentLimit(DriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG);
    // m_leftBackMotor.configSupplyCurrentLimit(DriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG);

    /* Config deadbands so that */
    m_leftBackMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND, DriveConstants.DRIVE_TIMEOUT_MS);

    /* PID for Front Motor Control in Teleop */
    try {
      if (m_pneumaticsSubsystem.m_isSpeedShiftHigh) {
        setRightMotorGains(true);
      } else {
        setRightMotorGains(false);
      }
    } catch (Exception e) {
      System.err.println("Error while trying to switch gains.");
    }

    /* PID for Back Motor Control in Tank Drive Vel */
    m_rightBackMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_rightBackMotor.config_kF(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.config_kP(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.config_kI(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.config_kD(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kD, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kPeakOutput,
        DriveConstants.DRIVE_TIMEOUT_MS);

    m_leftBackMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_leftBackMotor.config_kF(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kF, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.config_kP(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kP, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.config_kI(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kI, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.config_kD(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kD, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kPeakOutput,
        DriveConstants.DRIVE_TIMEOUT_MS);

    /* Reset Sensors for WPI_TalonFXs */
    resetEncoders();

    /* Configure the left Talon's selected sensor as local QuadEncoder */
    m_leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, // Local Feedback Source
        DriveConstants.PID_PRIMARY, // PID Index for Source [0, 1]
        DriveConstants.DRIVE_TIMEOUT_MS); // Configuration Timeout

    /* Configure the left back Talon's selected sensor as local QuadEncoder */
    m_leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, // Local Feedback Source
        DriveConstants.PID_PRIMARY, // PID Index for Source [0, 1]
        DriveConstants.DRIVE_TIMEOUT_MS); // Configuration Timeout

    /* Configure the right back Talon's selected sensor as local QuadEncoder */
    m_rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, // Local Feedback Source
        DriveConstants.PID_PRIMARY, // PID Index for Source [0, 1]
        DriveConstants.DRIVE_TIMEOUT_MS); // Configuration Timeout

    /*
     * Configure the Remote Talon's selected sensor as a remote sensor for the right
     * Talon
     */
    m_rightFrontMotor.configRemoteFeedbackFilter(m_leftFrontMotor.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, DriveConstants.REMOTE_0, // Source number [0, 1]
        DriveConstants.DRIVE_TIMEOUT_MS); // Configuration Timeout

    /* Diff Signal signal to be used for Distance */
    m_rightFrontMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.IntegratedSensor,
        DriveConstants.DRIVE_TIMEOUT_MS);

    /* Configure Diff [Sum of both QuadEncoders] to be used for Primary PID Index */
    m_rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, DriveConstants.PID_PRIMARY,
        DriveConstants.DRIVE_TIMEOUT_MS);

    /*
     * Configure the Pigeon IMU to the other Remote Slot available on the right
     * Talon
     */
    m_rightFrontMotor.configRemoteFeedbackFilter(m_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw,
        DriveConstants.REMOTE_1, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Config Remote1 to be used for Aux PID Index */
    m_rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, DriveConstants.PID_TURN,
        DriveConstants.DRIVE_TIMEOUT_MS);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    m_rightFrontMotor.configAuxPIDPolarity(DriveConstants.isAuxPIDInverted, DriveConstants.DRIVE_TIMEOUT_MS);

    /* Set status frame periods to ensure we don't have stale data */
    m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.DRIVE_TIMEOUT_MS);
    m_pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, DriveConstants.DRIVE_TIMEOUT_MS);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    m_leftFrontMotor.configPeakOutputForward(+1, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftFrontMotor.configPeakOutputReverse(-1, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configPeakOutputForward(+1, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.configPeakOutputReverse(-1, DriveConstants.DRIVE_TIMEOUT_MS);

    m_leftBackMotor.configPeakOutputForward(+1, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.configPeakOutputReverse(-1, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.configPeakOutputForward(+1, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.configPeakOutputReverse(-1, DriveConstants.DRIVE_TIMEOUT_MS);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    m_rightFrontMotor.configClosedLoopPeriod(DriveConstants.PID_PRIMARY, DriveConstants.CLOSED_LOOP_TIME_MS,
        DriveConstants.DRIVE_TIMEOUT_MS);

    m_rightFrontMotor.configClosedLoopPeriod(DriveConstants.PID_TURN, DriveConstants.CLOSED_LOOP_TIME_MS,
        DriveConstants.DRIVE_TIMEOUT_MS);

    m_leftBackMotor.configClosedLoopPeriod(DriveConstants.PID_PRIMARY, DriveConstants.CLOSED_LOOP_TIME_MS,
        DriveConstants.DRIVE_TIMEOUT_MS);

    m_rightBackMotor.configClosedLoopPeriod(DriveConstants.PID_PRIMARY, DriveConstants.CLOSED_LOOP_TIME_MS,
        DriveConstants.DRIVE_TIMEOUT_MS);

    /* Set up Differential Drive */
    m_driveTrain = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

    /* Set up Differential Drive Odometry. */
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        new Pose2d(0, 0, new Rotation2d()));

    /* Set up Orchestra */
    m_orchestra = new Orchestra();

    /* flip input so forward becomes back, etc */
    m_leftFrontMotor.setInverted(DriveConstants.isLeftMotorInverted);
    m_rightFrontMotor.setInverted(DriveConstants.isRightMotorInverted);
    m_leftBackMotor.setInverted(DriveConstants.isLeftMotorInverted);
    m_rightBackMotor.setInverted(DriveConstants.isRightMotorInverted);
    m_driveTrain.setRightSideInverted(DriveConstants.isRightArcadeInverted);

    /* Set up music for drive train */
    m_orchestra.addInstrument(m_leftBackMotor);
    m_orchestra.addInstrument(m_rightFrontMotor);
    m_orchestra.addInstrument(m_rightBackMotor);
    m_orchestra.addInstrument(m_leftFrontMotor);

    /* Create chooser to choose song to play */
    File songsDir = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/songs");
    System.err.println(songsDir.getPath());
    String[] songsStrings = songsDir.list();
    for (String songString : songsStrings) {
      m_songChooser.addOption(songString, songsDir.getAbsolutePath() + "/" + songString);
    }
    Shuffleboard.getTab("Songs").add(m_songChooser);

    /* Start counting time */
    m_lastTimeMs = System.currentTimeMillis();
  }

  @Override
  public void periodic() {
    updateTime();
    updateAngles();
    updatePosition();
    updateSmartDashboard();
  }

  /**
   * Passes subsystem needed.
   * 
   * @param subsystem Subsystem needed.
   */
  public void passRequiredSubsystem(Pneumatics subsystem) {
    m_pneumaticsSubsystem = subsystem;
  }

  public void updateTime() {
    m_lastTimeMs = m_currentTimeMs;
    m_currentTimeMs = System.currentTimeMillis();
    m_currentTimeSec = m_currentTimeMs / 1000;
    m_deltaTimeMs = m_currentTimeMs - m_lastTimeMs;
  }

  public void updateAngles() {
    m_lastAngleYaw = m_currentAngleYaw;
    m_currentAngleYaw = getGyroYaw();
  }

  public void updatePosition() {
    m_rightFrontMotorPos = m_rightFrontMotor.getSelectedSensorPosition();
    m_rightFrontMotorVel = m_rightFrontMotor.getSelectedSensorVelocity();

    m_lastRightPosTicks = m_currentRightPosTicks;
    m_lastLeftPosTicks = m_currentLeftPosTicks;
    m_currentRightPosTicks = m_rightFrontMotor.getSensorCollection().getIntegratedSensorPosition();
    m_currentLeftPosTicks = m_leftFrontMotor.getSensorCollection().getIntegratedSensorPosition();

    m_totalRightDistanceInches += ticksToInches(m_currentRightPosTicks - m_lastRightPosTicks);
    m_totalLeftDistanceInches += ticksToInches(m_currentLeftPosTicks - m_lastLeftPosTicks);

    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getDistanceInches(m_leftFrontMotor),
        -getDistanceInches(m_rightFrontMotor));
  }

  /**
   * Runs percent output control on the moving and steering of the drive train,
   * using the Differential Drive class to manage the two inputs
   */
  public void driveWithInput(double move, double steer) {
    m_driveTrain.arcadeDrive(move, steer);
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);
  }

  /**
   * Runs percent output control on the drive train while using an AUX PID for
   * rotation
   * 
   * @param targetPos  The position to drive to in units
   * @param targetGyro The angle to drive at in units
   */
  public void driveWithInputAux(double move, double targetGyro) {
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);

    m_rightFrontMotor.set(TalonFXControlMode.PercentOutput, move, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    m_driveTrain.feedWatchdog();
  }

  /**
   * Runs position PID. Position is absolute and displacement should be handled on
   * the command side.
   * 
   * @param targetPos  The position to drive to in units
   * @param targetGyro The angle to drive at in units
   */
  public void runDrivePositionPID(double targetPos, double targetGyro) {
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);

    m_rightFrontMotor.set(TalonFXControlMode.Position, targetPos, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    m_driveTrain.feedWatchdog();
  }

  /**
   * Runs velocity PID
   * 
   * @param targetVel  The velocity to drive at in units
   * @param targetGyro The angle to drive at in units
   */
  public void runDriveVelocityPID(double targetVel, double targetGyro) {
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);

    m_rightFrontMotor.set(TalonFXControlMode.Velocity, targetVel, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    m_driveTrain.feedWatchdog();
  }

  /**
   * Runs motion magic PID while driving straight
   * 
   * @param targetPos  The position to drive to in units
   * @param targetGyro The angle to drive at in units
   */
  public void runMotionMagicPID(double targetPos, double targetGyro) {
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_MOTION_MAGIC, DriveConstants.PID_PRIMARY);
    m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);

    m_rightFrontMotor.set(ControlMode.MotionMagic, targetPos, DemandType.AuxPID, targetGyro);
    m_leftFrontMotor.follow(m_rightFrontMotor, FollowerType.AuxOutput1);

    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    m_driveTrain.feedWatchdog();

  }

  /**
   * Runs a Turning PID to rotate a to a target angle
   * 
   * @param targetAngle target angle in degrees
   */
  public void runTurningPID(double targetAngle) {
    // double targetGyro = (targetAngle / 360) * DriveConstants.TICKS_PER_GYRO_REV;

    runDriveVelocityPID(0, targetAngle);
  }

  /**
   * Controls the left and right sides of the drive with velocity targets.
   *
   * @param leftSpeed  the commanded left speed
   * @param rightSpeed the commanded right speed
   */
  public void tankDriveVelocity(double leftSpeed, double rightSpeed) {
    // DifferentialDriveWheelSpeeds wheelSpeeds = new
    // DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    // ChassisSpeeds chassisSpeeds =
    // DriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);
    // double moveVelMPS = chassisSpeeds.vxMetersPerSecond;
    // double angleVelRad = chassisSpeeds.omegaRadiansPerSecond;
    // double angleVelDeg = Math.toDegrees(angleVelRad);

    // m_kinematicsTargetAngle += angleVelDeg * (m_deltaTime/1000);
    // m_kinematicsTargetAngle = MathUtil.clamp( m_kinematicsTargetAngle,
    // m_currentAngleYaw-(360),
    // m_currentAngleYaw+(360));
    // double targetGyro = (m_kinematicsTargetAngle / 360) *
    // DriveConstants.TICKS_PER_GYRO_REV;
    double moveVelLeft = inchesToTicks(metersToInches(leftSpeed)) / DriveConstants.SECONDS_TO_TICK_TIME;
    double moveVelRight = inchesToTicks(metersToInches(rightSpeed)) / DriveConstants.SECONDS_TO_TICK_TIME;

    // SmartDashboard.putNumber("Move Vel Left", moveVelLeft);
    // SmartDashboard.putNumber("Move Vel Right", moveVelRight);

    // runDriveVelocityPID(moveVel*2, targetGyro);

    m_rightBackMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
    m_leftBackMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);

    System.err.println(moveVelLeft);

    m_rightBackMotor.set(TalonFXControlMode.Velocity, moveVelRight);
    m_leftBackMotor.set(TalonFXControlMode.Velocity, moveVelLeft);
    m_leftFrontMotor.follow(m_leftBackMotor);
    m_rightFrontMotor.follow(m_rightBackMotor);

    m_driveTrain.feedWatchdog();
  }

  /**
   * Selects a song to play!
   * 
   * @param song The name of the song to be played
   */
  public void selectSong(String song) {
    SmartDashboard.putString("Selected Song", song);
    m_orchestra.loadMusic(song);
  }

  /*
   * Plays Music!
   */
  public void playSong() {
    m_orchestra.play();
  }

  /**
   * Resets the encoders for both motors.
   */
  public void resetEncoders() {
    m_leftFrontMotor.getSensorCollection().setIntegratedSensorPosition(0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightFrontMotor.getSensorCollection().setIntegratedSensorPosition(0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_leftBackMotor.getSensorCollection().setIntegratedSensorPosition(0, DriveConstants.DRIVE_TIMEOUT_MS);
    m_rightBackMotor.getSensorCollection().setIntegratedSensorPosition(0, DriveConstants.DRIVE_TIMEOUT_MS);

    m_totalLeftDistanceInches = 0;
    m_totalRightDistanceInches = 0;
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose to which to set the odometry.
   */
  public void setOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Resets the yaw of the pigeon
   */
  public void resetGyroYaw() {
    m_pigeon.setYaw(0);
    m_pigeon.setAccumZAngle(0);
    resetGyroAngles();
  }

  /**
   * Add docs here
   */
  public void resetGyroAngles() {
    m_lastAngleYaw = 0;
    m_currentAngleYaw = 0;
  }

  /**
   * Returns the current yaw of the pigeon
   */
  public double getGyroYaw() {

    double[] ypr = new double[3];

    m_pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  /**
   * Returns the current pitch of the pigeon
   */
  public double getGyroPitch() {
    double[] ypr = new double[3];

    m_pigeon.getYawPitchRoll(ypr);
    return ypr[1];
  }

  /**
   * Returns the current roll of the pigeon
   */
  public double getGyroRoll() {
    double[] ypr = new double[3];

    m_pigeon.getYawPitchRoll(ypr);
    return ypr[2];
  }

  public GyroBase getGyroInterface() {
    return new GyroBase(){
    
      @Override
      public void close() throws Exception {
        // TODO Auto-generated method stub
      }
    
      @Override
      public void reset() {
        // TODO Auto-generated method stub
        resetGyroYaw();
      }
    
      @Override
      public double getRate() {
        // TODO Auto-generated method stub
        return getTurnRate();
      }
    
      @Override
      public double getAngle() {
        // TODO Auto-generated method stub
        return getGyroYaw();
      }
    
      @Override
      public void calibrate() {
        // TODO Auto-generated method stub
      }
    };
  }

  // lol
  // sko
  // ridge
  // brayden=bad coder

  /**
   * Returns the heading of the robot
   * 
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(getGyroYaw(), 360);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double deltaYaw = m_currentAngleYaw - m_lastAngleYaw;
    double turnRate = 1000 * deltaYaw / m_deltaTimeMs;
    return turnRate;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * 
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns current wheel speeds of robot.
   * 
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocityInchesPerSecond(m_leftBackMotor),
        -getVelocityInchesPerSecond(m_rightBackMotor));
  }

  /**
   * Gets the encoder value (position) of a motor
   * 
   * @param falcon The motor to get the position of
   * @return The position of the motor in inches
   */
  public double getDistanceInches(WPI_TalonFX falcon) {
    return ticksToInches(falcon.getSensorCollection().getIntegratedSensorPosition());
  }

  /**
   * Gets the encoder value (velocity) of a motor
   * 
   * @param falcon The motor to get the velocity of
   * @return The velocity of the motor in inches per second
   */
  public double getVelocityInchesPerSecond(WPI_TalonFX falcon) {
    return ticksToInches(
        falcon.getSensorCollection().getIntegratedSensorPosition() / DriveConstants.TICK_TIME_TO_SECONDS);
  }

  /**
   * Converts a value in ticks to inches.
   * 
   * @param ticks The value in ticks to convert
   * @return The converted value in inches
   */
  public double ticksToInches(double ticks) {
    if (m_pneumaticsSubsystem.m_isSpeedShiftHigh) {
      return ticks * DriveConstants.INCHES_PER_TICK_HIGH;
    } else {
      return ticks * DriveConstants.INCHES_PER_TICK_LOW;
    }
  }

  /**
   * Converts a value in inches to ticks.
   * 
   * @param inches The value in inches to convert
   * @return The converted value in ticks
   */
  public double inchesToTicks(double inches) {
    if (m_pneumaticsSubsystem.m_isSpeedShiftHigh) {
      return inches * DriveConstants.TICKS_PER_INCH_HIGH;
    } else {
      return inches * DriveConstants.TICKS_PER_INCH_LOW;
    }
  }

  /**
   * Converts a value in inches to meters.
   * 
   * @param inches The value in inches to convert
   * @return The converted value in meters
   */
  public double inchesToMeters(double inches) {
    return inches * DriveConstants.METERS_PER_INCH;
  }

  /**
   * Converts a value in meters to inches.
   * 
   * @param meters The value in meters to convert
   * @return The converted value in inches
   */
  public double metersToInches(double meters) {
    return meters * DriveConstants.INCHES_PER_METER;
  }

  public void setRightMotorGains(boolean isHighGear) {
    if (!isHighGear) {
      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_VELOCITY, m_gainsVelocityLow.m_kPeakOutput,
          DriveConstants.DRIVE_TIMEOUT_MS);

      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_TURNING, m_gainsTurningLow.m_kF, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_TURNING, m_gainsTurningLow.m_kP, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_TURNING, m_gainsTurningLow.m_kI, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_TURNING, m_gainsTurningLow.m_kD, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_TURNING, m_gainsTurningLow.m_kPeakOutput,
          DriveConstants.DRIVE_TIMEOUT_MS);

      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_DISTANCE, m_gainsDistanceLow.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_DISTANCE, m_gainsDistanceLow.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_DISTANCE, m_gainsDistanceLow.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_DISTANCE, m_gainsDistanceLow.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_DISTANCE, m_gainsDistanceLow.m_kPeakOutput,
          DriveConstants.DRIVE_TIMEOUT_MS);

      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_MOTION_MAGIC, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicLow.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicLow.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicLow.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicLow.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_MOTION_MAGIC,
          m_gainsMotionMagicLow.m_kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionCruiseVelocity(DriveConstants.DRIVE_CRUISE_VELOCITY,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionAcceleration(DriveConstants.DRIVE_ACCELERATION, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionSCurveStrength(0, DriveConstants.DRIVE_TIMEOUT_MS);
    } else {
      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_VELOCITY, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_VELOCITY, m_gainsVelocityHigh.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_VELOCITY, m_gainsVelocityHigh.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_VELOCITY, m_gainsVelocityHigh.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_VELOCITY, m_gainsVelocityHigh.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_VELOCITY, m_gainsVelocityHigh.m_kPeakOutput,
          DriveConstants.DRIVE_TIMEOUT_MS);

      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_TURNING, DriveConstants.PID_TURN);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_TURNING, m_gainsTurningHigh.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_TURNING, m_gainsTurningHigh.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_TURNING, m_gainsTurningHigh.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_TURNING, m_gainsTurningHigh.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_TURNING, m_gainsTurningHigh.m_kPeakOutput,
          DriveConstants.DRIVE_TIMEOUT_MS);

      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_DISTANCE, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_DISTANCE, m_gainsDistanceHigh.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_DISTANCE, m_gainsDistanceHigh.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_DISTANCE, m_gainsDistanceHigh.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_DISTANCE, m_gainsDistanceHigh.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_DISTANCE, m_gainsDistanceHigh.m_kPeakOutput,
          DriveConstants.DRIVE_TIMEOUT_MS);

      m_rightFrontMotor.selectProfileSlot(DriveConstants.SLOT_MOTION_MAGIC, DriveConstants.PID_PRIMARY);
      m_rightFrontMotor.config_kF(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicHigh.m_kF,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kP(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicHigh.m_kP,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kI(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicHigh.m_kI,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.config_kD(DriveConstants.SLOT_MOTION_MAGIC, m_gainsMotionMagicHigh.m_kD,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configClosedLoopPeakOutput(DriveConstants.SLOT_MOTION_MAGIC,
          m_gainsMotionMagicHigh.m_kPeakOutput, DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionCruiseVelocity(DriveConstants.DRIVE_CRUISE_VELOCITY_HIGH,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionAcceleration(DriveConstants.DRIVE_ACCELERATION_HIGH,
          DriveConstants.DRIVE_TIMEOUT_MS);
      m_rightFrontMotor.configMotionSCurveStrength(0, DriveConstants.DRIVE_TIMEOUT_MS);
    }
  }

  /**
   * Sets Motors to a NeutralMode.
   * 
   * @param mode NeutralMode to set motors to
   */
  public void setDriveTrainNeutralMode(NeutralMode mode) {
    m_leftFrontMotor.setNeutralMode(mode);
    m_rightFrontMotor.setNeutralMode(mode);
    m_leftBackMotor.setNeutralMode(mode);
    m_rightBackMotor.setNeutralMode(mode);
  }

  public void updateSmartDashboard() {
    try {
      // SmartDashboard.putNumber("Pigeon Yaw", getGyroYaw());
      // SmartDashboard.putNumber("Pigeon Pitch", getGyroPitch());
      // SmartDashboard.putNumber("Pigeon Roll", getGyroRoll());

      SmartDashboard.putData("Pigeon Gyro", m_pigeonGyro);
      SmartDashboard.putData("Drive Train", m_driveTrain);


      //SmartDashboard.putNumber("Left Front Output", m_leftFrontMotor.get());
      //SmartDashboard.putNumber("Right Front Output", m_rightFrontMotor.get());
      //SmartDashboard.putNumber("Left Back Output", m_leftBackMotor.get());
      //SmartDashboard.putNumber("Right Back Output", m_rightBackMotor.get());

      double leftRPM = ticksToInches(m_leftFrontMotor.getSensorCollection().getIntegratedSensorVelocity());
      double rightRPM = ticksToInches(m_rightFrontMotor.getSensorCollection().getIntegratedSensorVelocity());

      //SmartDashboard.putNumber("Left Motor RPM", leftRPM);
      //SmartDashboard.putNumber("Right Motor RPM", rightRPM);

      //SmartDashboard.putNumber("Left Back Motor Velocity Raw", m_leftBackMotor.getSelectedSensorVelocity());
      //SmartDashboard.putNumber("Right Back Motor Velocity Raw", m_rightBackMotor.getSelectedSensorVelocity());
      //SmartDashboard.putNumber("Left Motor Position Raw", m_leftFrontMotor.getSelectedSensorPosition());
      //SmartDashboard.putNumber("Right Motor Position Raw", m_rightFrontMotor.getSelectedSensorPosition(0));
      
      //SmartDashboard.putNumber("Right Motor Velocity Int Sensor", m_rightFrontMotor.getSensorCollection().getIntegratedSensorVelocity());
      //SmartDashboard.putNumber("Left Motor Velocity Int Sensor", m_leftFrontMotor.getSensorCollection().getIntegratedSensorVelocity());
      //SmartDashboard.putNumber("Left Motor Pos Inches", getDistanceInches(m_rightFrontMotor));
      //SmartDashboard.putNumber("Right Motor Pos Inches", getDistanceInches(m_leftFrontMotor));

      /*SmartDashboard.putNumber("Right Front Velocity", m_rightFrontMotor.getSensorCollection().getIntegratedSensorVelocity());
      SmartDashboard.putNumber("Left Front Velocity", m_leftFrontMotor.getSensorCollection().getIntegratedSensorVelocity());
      SmartDashboard.putNumber("Right Back Velocity", m_rightBackMotor.getSensorCollection().getIntegratedSensorVelocity());
      SmartDashboard.putNumber("Left Back Velocity", m_leftBackMotor.getSensorCollection().getIntegratedSensorVelocity());
      */

      //SmartDashboard.putNumber("Right Motor Temp", m_rightFrontMotor.getTemperature());
      //SmartDashboard.putNumber("Left Motor Temp", m_leftFrontMotor.getTemperature());

      //SmartDashboard.putNumber("Right Front Motor Current Supply", m_rightFrontMotor.getSupplyCurrent());
      //SmartDashboard.putNumber("Left Front Motor Current Supply", m_leftFrontMotor.getSupplyCurrent());
      //SmartDashboard.putNumber("Right Back Motor Current Supply", m_rightBackMotor.getSupplyCurrent());
      //SmartDashboard.putNumber("Left Back Motor Current Supply", m_leftBackMotor.getSupplyCurrent());
      
      //SmartDashboard.putNumber("Right Front Motor Current Stator ", m_rightFrontMotor.getStatorCurrent());
      //SmartDashboard.putNumber("Left Front Motor Current Stator", m_leftFrontMotor.getStatorCurrent());
      //SmartDashboard.putNumber("Right Back Motor Current Stator ", m_rightBackMotor.getStatorCurrent());
      //SmartDashboard.putNumber("Left Back Motor Current Stator", m_leftBackMotor.getStatorCurrent());

      //SmartDashboard.putNumber("PID 0 Error", m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_PRIMARY));
      //SmartDashboard.putNumber("PID 1 Error", m_rightFrontMotor.getClosedLoopError(DriveConstants.PID_TURN));
      //SmartDashboard.putNumber("PID 0 Target", m_rightFrontMotor.getClosedLoopTarget(DriveConstants.PID_PRIMARY));
      //SmartDashboard.putNumber("PID 1 Target", m_rightFrontMotor.getClosedLoopTarget(DriveConstants.PID_TURN));
      //SmartDashboard.putNumber("PID 0 Pos", m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_PRIMARY));
      //SmartDashboard.putNumber("PID 1 Pos", m_rightFrontMotor.getSelectedSensorPosition(DriveConstants.PID_TURN));

      //SmartDashboard.putString("Odometry Values Meters", getPose().toString());
      //SmartDashboard.putNumber("Odometry Heading", getHeading());

      SmartDashboard.putNumber("Time Seconds", m_currentTimeSec);
      SmartDashboard.putNumber("Delta Time", m_deltaTimeMs);

      if (m_currentSong != m_songChooser.getSelected()){
        m_currentSong = m_songChooser.getSelected();
        selectSong(m_currentSong);
        //System.err.println(m_currentSong);
      }
    } catch (Exception e) {
      System.err.println("Error while using Drive SmartDashboard");
      // e.printStackTrace(System.err);
    }
  }
}
