package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // ====== CHANGE THESE TO YOUR REAL CAN IDs ======

  private static final int NEO_ID = 14;      // Top shooter roller
  private static final int KRAKEN_ID = 13;   // Bottom intake / feeder roller

  /** Nominal voltages at 12 V; scaled by battery/12 for consistent behavior when sagged. */
  private static final double FEED_VOLTAGE_NOMINAL = 7.0;
  private static final double SHOOT_VOLTAGE_NOMINAL = 9.0;

  /** Nominal bus voltage for scaling (V). */
  private static final double NOMINAL_BUS_V = 12.0;

  private final SparkMax feed_motor = new SparkMax(NEO_ID, MotorType.kBrushless);
  private final TalonFX shoot_motor = new TalonFX(KRAKEN_ID);

  private final VoltageOut krakenVoltage = new VoltageOut(0.0);
  private final VelocityVoltage shooterVelocity = new VelocityVoltage(0.0);

  /** Open-loop ramp period (s) for normal operation; 0 during SysId so data is not distorted. */
  private static final double OPEN_LOOP_RAMP_PERIOD_S = 0.12;

  /** SysId routine for characterizing the Kraken shooter motor. */
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,        // default 1 V/s ramp rate
          Volts.of(4), // 4 V dynamic step
          null,        // default 10 s timeout
          state -> SignalLogger.writeString("SysIdShooter_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> shoot_motor.setControl(krakenVoltage.withOutput(output.in(Volts))),
          null,
          this));

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

    // ===== Ramp for smooth acceleration (disabled during SysId to avoid corrupting data) =====
    shoot_motor.getConfigurator().apply(
        new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(OPEN_LOOP_RAMP_PERIOD_S));

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
  }

  /** Scale factor from current bus voltage so intake/shoot voltages stay consistent when battery sags. */
  private static double voltageScale() {
    double v = RobotController.getBatteryVoltage();
    return Math.min(1.0, Math.max(0.5, v / NOMINAL_BUS_V));
  }

  /** Sets shooter and feeder to given nominal voltages (scaled by bus voltage). */
  private void setVoltages(double shootVNominal, double feedVNominal) {
    double scale = voltageScale();
    shoot_motor.setControl(krakenVoltage.withOutput(shootVNominal * scale));
    feed_motor.setVoltage(feedVNominal * scale);
  }

  // ===== LT Intake Mode (voltage-compensated) =====

  public void runIntake() {
    setVoltages(SHOOT_VOLTAGE_NOMINAL, -FEED_VOLTAGE_NOMINAL);
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
    feed_motor.setVoltage(FEED_VOLTAGE_NOMINAL * voltageScale());
  }

  /** Stops only the feeder motor (shooter wheel unchanged). */
  public void stopFeeder() {
    feed_motor.setVoltage(0.0);
  }

  /** Reverse feeder (B button). */
  public void runFeedReverse() {
    feed_motor.setVoltage(-FEED_VOLTAGE_NOMINAL * voltageScale());
  }

  /** Reverse whole system of taking in balls (A button). */
  public void runSystemReverse() {
    double scale = voltageScale();
    feed_motor.setVoltage(FEED_VOLTAGE_NOMINAL * scale);
    shoot_motor.setControl(krakenVoltage.withOutput(-SHOOT_VOLTAGE_NOMINAL * scale));
  }

  // ===== Mechanism feedback (optional beam break / sensor) =====

  /**
   * Whether a game piece is detected (e.g. beam break at shooter entry).
   * Wire a beam break or sensor to a DIO and override this in a subclass or set via setter.
   * Default: false (no sensor installed).
   */
  public boolean hasGamePiece() {
    return false;
  }

  // ===== Stop Everything =====
  public void stop() {
    shoot_motor.setControl(krakenVoltage.withOutput(0.0));
    feed_motor.setVoltage(0.0);
  }

  // ===== SysId characterization =====

  /**
   * Sets open-loop ramp period on the Kraken. Use 0 during SysId so quasistatic/dynamic
   * voltage profiles are not filtered; restore normal ramp afterward.
   */
  private void setOpenLoopRampPeriod(double periodSeconds) {
    shoot_motor.getConfigurator().apply(
        new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(periodSeconds));
  }

  /** Runs SysId quasistatic test in the given direction. Ramp disabled for test, restored after. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine
        .quasistatic(direction)
        .beforeStarting(() -> setOpenLoopRampPeriod(0))
        .finallyDo(() -> setOpenLoopRampPeriod(OPEN_LOOP_RAMP_PERIOD_S));
  }

  /** Runs SysId dynamic test in the given direction. Ramp disabled for test, restored after. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine
        .dynamic(direction)
        .beforeStarting(() -> setOpenLoopRampPeriod(0))
        .finallyDo(() -> setOpenLoopRampPeriod(OPEN_LOOP_RAMP_PERIOD_S));
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
   * @return Applied voltage (V)
   */
  public double getShooterVoltage() {
    return shoot_motor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * Gets the supply (bus) voltage to the shooter motor.
   * @return Supply voltage (V)
   */
  public double getShooterSupplyVoltage() {
    return shoot_motor.getSupplyVoltage().getValueAsDouble();
  }
}
