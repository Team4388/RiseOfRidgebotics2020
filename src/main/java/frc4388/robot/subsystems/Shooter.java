/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.CSV;
import frc4388.utility.Trims;

public class Shooter extends SubsystemBase {

  public final WPI_TalonFX m_shooterFalconLeft = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_BALLER_ID);
  public final WPI_TalonFX m_shooterFalconRight = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_BALLER_FOLLOWER_ID);

  public static class ShooterTableEntry {
    public Double distance, hoodExt, drumVelocity, centerDisplacement;
  }

  private ShooterTableEntry[] m_shooterTable;

  public boolean m_isDrumReady = false;
  public double m_fireVel;

  public Trims shooterTrims;

  public ShooterHood m_shooterHoodSubsystem;
  public ShooterAim m_shooterAimSubsystem;

  /*
   * Creates a new Shooter subsystem, with the drum shooter and the angle adjuster.
   */
  public Shooter() {
    // Testing purposes reseting gyros
    // resetGyroAngleAdj();
    shooterTrims = new Trims(0, 0);
    // SmartDashboard.putNumber("Velocity Target", 10000);
    // SmartDashboard.putNumber("Angle Target", 3);

    // LEFT FALCON
    m_shooterFalconLeft.configFactoryDefault();
    m_shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
    m_shooterFalconLeft.setInverted(true);
    m_shooterFalconLeft.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);

    // RIGHT FALCON
    m_shooterFalconRight.configFactoryDefault();
    m_shooterFalconRight.setNeutralMode(NeutralMode.Coast);
    m_shooterFalconRight.setInverted(false);
    m_shooterFalconRight.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconRight.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    setShooterGains();

    final int closedLoopTimeMs = 1;
    // LEFT FALCON
    m_shooterFalconLeft.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);

    // RIGHT FALCON
    // m_shooterFalconRight.configPeakOutputForward(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconRight.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconRight.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconRight.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);

    try {
      m_shooterTable = new CSV<>(ShooterTableEntry::new) {
        private final Pattern parentheses = Pattern.compile("\\([^\\)]*+\\)");

        @Override
        protected String headerSanitizer(final String header) {
          return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
        }
      }.read(new File(Filesystem.getDeployDirectory(), "Robot Data - Distances.csv").toPath());
      new Thread(() -> System.out.println(CSV.ReflectionTable.create(m_shooterTable))).start();
    } catch (final IOException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try {
      SmartDashboard.putNumber("Drum Velocity", m_shooterFalconLeft.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Drum Velocity CSV", m_fireVel);
      SmartDashboard.putNumber("Shooter Temp C", m_shooterFalconLeft.getTemperature());
      SmartDashboard.putNumber("Shooter Current", m_shooterFalconLeft.getSupplyCurrent());
      SmartDashboard.putBoolean("Drum Ready", m_isDrumReady);
    } catch (final Exception e) {

    }
  }

  /**
   * Passes subsystem needed.
   * @param subsystem Subsystem needed.
   */
  public void passRequiredSubsystem(ShooterHood subsystem0, ShooterAim subsystem1) {
    m_shooterHoodSubsystem = subsystem0;
    m_shooterAimSubsystem = subsystem1;
  }

  public double addFireVel() {
    return m_fireVel;
  }

  /**
   * Runs drum shooter motor.
   * @param speed Speed to set the motor at
   */
  public void runDrumShooter(double speed) {
    m_shooterFalconLeft.set(TalonFXControlMode.PercentOutput, speed);
    m_shooterFalconRight.follow(m_shooterFalconLeft);
    // m_shooterFalconRight.set(TalonFXControlMode.PercentOutput, speed);
  }

  /**
   * Configures gains for shooter PID.
   */
  public void setShooterGains() {
    m_shooterFalconLeft.selectProfileSlot(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_PID_LOOP_IDX);
    m_shooterFalconLeft.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.DRUM_SHOOTER_GAINS.m_kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.DRUM_SHOOTER_GAINS.m_kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.DRUM_SHOOTER_GAINS.m_kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.DRUM_SHOOTER_GAINS.m_kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
  }

  /**
   * Runs drum shooter velocity PID.
   * @param targetVel Target velocity to run motor at
   */
  public void runDrumShooterVelocityPID(double targetVel) {
    // System.out.println("Target Velocity" + targetVel);
    m_shooterFalconLeft.set(TalonFXControlMode.Velocity, targetVel); // Init PID
    m_shooterFalconRight.follow(m_shooterFalconLeft);
  }

  public Double getCenterDisplacement(final Double distance) {
    return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.centerDisplacement).doubleValue();
  }

  public Double getVelocity(final Double distance) {
    return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.drumVelocity).doubleValue();
  }

  public Double getHood(final Double distance) {
    return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.hoodExt).doubleValue();
  }

  private static <E> Number linearInterpolate(final E[] table, final Number lookupValue, final Function<E, Number> lookupGetter, final Function<E, Number> targetGetter) {
    final Map.Entry<Integer, E> closestEntry = lookup(table, lookupValue.doubleValue(), lookupGetter, false).orElse(Map.entry(table.length - 1, table[table.length - 1]));
    final E closestRecord = closestEntry.getValue();
    final int closestRecordIndex = closestEntry.getKey();
    final E neighborRecord = table[lookupValue.doubleValue() <= lookupGetter.apply(closestRecord).doubleValue() ? Math.max(closestRecordIndex == 0 ? 1 : 0, closestRecordIndex - 1) : Math.min(closestRecordIndex + 1, table.length - (closestRecordIndex == table.length - 1 ? 2 : 1))];
    return lerp2(lookupValue, lookupGetter.apply(closestRecord), targetGetter.apply(closestRecord), lookupGetter.apply(neighborRecord), targetGetter.apply(neighborRecord));
  }

  private static <E> Optional<Map.Entry<Integer, E>> lookup(final E[] table, final Number value, final Function<E, Number> valueGetter, final boolean exactMatch) {
    final Optional<Map.Entry<Integer, E>> match = IntStream.range(0, table.length).mapToObj(i -> Map.entry(i, table[i])).min(Comparator.comparingDouble(e -> Math.abs(valueGetter.apply(e.getValue()).doubleValue() - value.doubleValue())));
    return !exactMatch || match.map(e -> valueGetter.apply(e.getValue()).equals(value)).orElse(false) ? match : Optional.empty();
  }

  private static Number lerp2(final Number x, final Number x0, final Number y0, final Number x1, final Number y1) {
    final Number f = (x.doubleValue() - x0.doubleValue()) / (x1.doubleValue() - x0.doubleValue());
    return (1.0 - f.doubleValue()) * y0.doubleValue() + f.doubleValue() * y1.doubleValue();
  }
}