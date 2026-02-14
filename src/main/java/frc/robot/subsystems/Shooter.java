package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  // ====== CHANGE THESE TO YOUR REAL CAN IDs ======
  private static final int TOP_CAN_ID = 15;      // Top shooter roller
  private static final int BOTTOM_CAN_ID = 16;   // Bottom intake / feeder roller

  // ====== L2 INTAKE COMBO ======
  // Bottom pulls note in
  // Top lightly assists
  private static final double INTAKE_TOP_VOLTS = -2.0;
  private static final double INTAKE_BOTTOM_VOLTS = 6.0;

  // ====== TRIANGLE SHOOT/FEED COMBO ======
  // Top launches
  // Bottom feeds upward
  private static final double SHOOT_TOP_VOLTS = 10.0;
  private static final double FEED_BOTTOM_VOLTS = 4.0;

  private final TalonFX topMotor = new TalonFX(TOP_CAN_ID);
  private final TalonFX bottomMotor = new TalonFX(BOTTOM_CAN_ID);

  private final VoltageOut topVoltage = new VoltageOut(0.0);
  private final VoltageOut bottomVoltage = new VoltageOut(0.0);

  public Shooter() {

    // ===== Motor Output Config =====
    topMotor.getConfigurator().apply(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)
    );

    bottomMotor.getConfigurator().apply(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)
    );

    // ===== Ramp for smooth acceleration =====
    var ramp = new OpenLoopRampsConfigs()
        .withVoltageOpenLoopRampPeriod(0.12);

    topMotor.getConfigurator().apply(ramp);
    bottomMotor.getConfigurator().apply(ramp);

    // ===== Current Limits (safe Kraken defaults) =====
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(55.0)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(110.0);

    topMotor.getConfigurator().apply(limits);
    bottomMotor.getConfigurator().apply(limits);

    stop();
  }

  // ===== L2 Intake Mode =====
  public void runIntakeCombo() {
    setVoltages(INTAKE_TOP_VOLTS, INTAKE_BOTTOM_VOLTS);
  }

  // ===== Triangle Shoot Mode =====
  public void runShootFeedCombo() {
    setVoltages(SHOOT_TOP_VOLTS, FEED_BOTTOM_VOLTS);
  }

  // ===== Core Voltage Setter =====
  public void setVoltages(double topVolts, double bottomVolts) {
    topMotor.setControl(topVoltage.withOutput(topVolts));
    bottomMotor.setControl(bottomVoltage.withOutput(bottomVolts));
  }

  // ===== Stop Everything =====
  public void stop() {
    topMotor.setControl(topVoltage.withOutput(0.0));
    bottomMotor.setControl(bottomVoltage.withOutput(0.0));
  }
}
