package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  // ====== CHANGE THESE TO YOUR REAL CAN IDs ======

  private static final int neo_id = 15;      // Top shooter roller
  private static final int kraken_id = 16;   // Bottom intake / feeder roller

  private static final double feed_voltage = 7.0;
  private static final double feed_reversed_voltage = -7.0;

  private static final double shoot_voltage = -9.0;
  private static final double shoot_reversed_voltage = 9.0;

  private final SparkMax feed_motor = new SparkMax(neo_id, MotorType.kBrushless);
  private final TalonFX shoot_motor = new TalonFX(kraken_id);

  private final VoltageOut krakenVoltage = new VoltageOut(0.0);

  public Shooter() {

    // ===== Motor Output Config =====
    shoot_motor.getConfigurator().apply(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)
    );

    // ===== Ramp for smooth acceleration =====
    var ramp = new OpenLoopRampsConfigs()
        .withVoltageOpenLoopRampPeriod(0.12);

    shoot_motor.getConfigurator().apply(ramp);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(55.0)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(110.0);

    shoot_motor.getConfigurator().apply(limits);

  // Configure spark settings

  SparkMaxConfig config = new SparkMaxConfig();
  config.idleMode(IdleMode.kBrake);
  config.inverted(false);
  config.smartCurrentLimit(40);  // 40A limit for NEO

// Apply configuration to motor
  feed_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ===== Current Limits (safe Kraken defaults) ====

    stop();
  }

  // ===== L2 Intake Mode =====

  public void runIntake() {
    setVoltages(shoot_voltage, feed_voltage);
  }

  // ===== L1 Feed =====

  public void runFeed() {
    setVoltages(0, feed_voltage);
  }

  //  ===== R2 Shooter Mode =====

   public void runShoot(double volt) { // double volt comes from the limelight
    setVoltages(volt, 0);
  }

  // ===== L1 + square  =====

  public void runIntakeReverse() {
    setVoltages(shoot_reversed_voltage, 0);
  }

  // ===== L1 + circle =====

  public void runFeedReverse() {
    setVoltages(feed_reversed_voltage, 0);
  }

  // ===== Core Voltage Setter =====
  public void setVoltages(double kraken_v, double neo_v) {
    shoot_motor.setControl(krakenVoltage.withOutput(kraken_v));
    feed_motor.setVoltage(neo_v);
  }

  // ===== Stop Everything =====
  public void stop() {
    shoot_motor.setControl(krakenVoltage.withOutput(0.0));
    feed_motor.setVoltage(0.0);
  }

  // ===== Getters for Telemetry and Ready Detection =====

  /**
   * Gets the current velocity of the shooter motor (Kraken X60).
   * @return Velocity in rotations per second
   */
  public double getShooterVelocity() {
    return shoot_motor.getVelocity().getValueAsDouble();
  }

  /**
   * Gets the current voltage being applied to the shooter motor.
   * @return Applied voltage
   */
  public double getShooterVoltage() {
    return shoot_motor.getMotorVoltage().getValueAsDouble();
  }
}
