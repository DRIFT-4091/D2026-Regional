package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // ====== CHANGE THESE TO YOUR REAL CAN IDs ======

  private static final int neo_id = 14;      // Top shooter roller
  private static final int kraken_id = 13;   // Bottom intake / feeder roller

  private static final double feed_voltage = 7.0;

  private static final double shoot_voltage = 9.0;

  private final SparkMax feed_motor = new SparkMax(neo_id, MotorType.kBrushless);
  private final TalonFX shoot_motor = new TalonFX(kraken_id);

  private final VoltageOut krakenVoltage = new VoltageOut(0.0);
  private final VelocityVoltage shooterVelocity = new VelocityVoltage(0.0);

  public Shooter() {

    // ===== Motor Output Config =====
    shoot_motor.getConfigurator().apply(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)
    );

    // ===== Velocity PID (holds RPM under load; tune kV and kP on robot) =====
    shoot_motor.getConfigurator().apply(
        new Slot0Configs()
            .withKP(Constants.Shooter.SHOOTER_VEL_KP)
            .withKI(Constants.Shooter.SHOOTER_VEL_KI)
            .withKD(Constants.Shooter.SHOOTER_VEL_KD)
            .withKS(Constants.Shooter.SHOOTER_VEL_KS)
            .withKV(Constants.Shooter.SHOOTER_VEL_KV)
    );

    // ===== Ramp for smooth acceleration =====
    var ramp = new OpenLoopRampsConfigs()
        .withVoltageOpenLoopRampPeriod(0.12);

    shoot_motor.getConfigurator().apply(ramp);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(40.0)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(110.0);

    shoot_motor.getConfigurator().apply(limits);

    // ===== Configure spark settings =====
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.smartCurrentLimit(40);  // 40A limit for NEO
    feed_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stop();

    stop();
  }

  /** Sets shooter and feeder to given voltages (for intake/score). */
  private void setVoltages(double shootV, double feedV) {
    shoot_motor.setControl(krakenVoltage.withOutput(shootV));
    feed_motor.setVoltage(feedV);
  }

  // ===== LT Intake Mode =====

  public void runIntake() {
    setVoltages(shoot_voltage, -feed_voltage);
  }

  //  ===== RT Shooter Mode (velocity control: holds RPM under load) =====

  /**
   * Runs shooter at target velocity (rotations per second). Holds RPM so ball friction doesn't limit throughput.
   * If targetRPS &lt;= 0 (e.g. no AprilTag), stops the shooter.
   */
  public void runShoot(double targetRPS) {
    if (targetRPS <= 0) {
      stop();
      return;
    }
    shoot_motor.setControl(shooterVelocity.withVelocity(targetRPS));
  }

  /** Get ball to the shooter (y button). */
  public void runFeed() {
    feed_motor.setVoltage(feed_voltage);
  }

  /** Stops only the feeder motor (shooter wheel unchanged). */
  public void stopFeeder() {
    feed_motor.setVoltage(0.0);
  }

  /** Reverse feeder (B button). */
  public void runFeedReverse() {
    feed_motor.setVoltage(-feed_voltage);
  }

  /** Reverse whole system of taking in balls (A button). */
  public void runSystemReverse() {
    feed_motor.setVoltage(feed_voltage);
    shoot_motor.setControl(krakenVoltage.withOutput(-shoot_voltage));
  }

  // ===== Stop Everything =====
  public void stop() {
    shoot_motor.setControl(krakenVoltage.withOutput(0.0));
    feed_motor.setVoltage(0.0);
  }

  // ===== Getters for Telemetry =====

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
