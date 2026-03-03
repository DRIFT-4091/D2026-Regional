package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;
import frc.robot.vision.MegaTag;

/**
 * Publishes telemetry to NetworkTables in three sections: Robot, Limelight, Shooter.
 * Also runs MegaTag2 vision pose updates (with fallback when pose is bad).
 * Units are included in the key names where applicable.
 */
public class Telemetry {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final MegaTag megaTag;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot: per-module steering angle (deg), CANcoder angle (deg), drive velocity (rps), supply voltage (V) */
    private final NetworkTable robotTable = inst.getTable("Robot");
    private final DoublePublisher[] modSteeringAngle_deg = new DoublePublisher[4];
    private final DoublePublisher[] modCANcoderAngle_deg = new DoublePublisher[4];
    private final DoublePublisher[] modDriveVelocity_rps = new DoublePublisher[4];
    private final DoublePublisher[] modSupplyVoltage_V = new DoublePublisher[4];

    /* Limelight: tid (dimensionless), tx/ty (deg), ta (%) */
    private final NetworkTable limelightTable = inst.getTable("Limelight");
    private final DoublePublisher limelightTid = limelightTable.getDoubleTopic("tid").publish();
    private final DoublePublisher limelightTx_deg = limelightTable.getDoubleTopic("tx_deg").publish();
    private final DoublePublisher limelightTy_deg = limelightTable.getDoubleTopic("ty_deg").publish();
    private final DoublePublisher limelightTa_percent = limelightTable.getDoubleTopic("ta_percent").publish();

    /* Shooter: speed (RPS), supply voltage (V) */
    private final NetworkTable shooterTable = inst.getTable("Shooter");
    private final DoublePublisher shooterSpeed_rps = shooterTable.getDoubleTopic("Speed_rps").publish();
    private final DoublePublisher shooterSupplyVoltage_V = shooterTable.getDoubleTopic("SupplyVoltage_V").publish();

    private static final double ROTATIONS_TO_DEGREES = 360.0;

    /**
     * Constructs telemetry with references to drivetrain, shooter, and MegaTag for publishing.
     */
    public Telemetry(CommandSwerveDrivetrain drivetrain, Shooter shooter, MegaTag megaTag) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.megaTag = megaTag;

        for (int i = 0; i < 4; i++) {
            NetworkTable modTable = robotTable.getSubTable("Module" + i);
            modSteeringAngle_deg[i] = modTable.getDoubleTopic("SteeringAngle_deg").publish();
            modCANcoderAngle_deg[i] = modTable.getDoubleTopic("CANcoderAngle_deg").publish();
            modDriveVelocity_rps[i] = modTable.getDoubleTopic("DriveVelocity_rps").publish();
            modSupplyVoltage_V[i] = modTable.getDoubleTopic("SupplyVoltage_V").publish();
        }
    }

    /**
     * Publishes Robot, Limelight, and Shooter data to NetworkTables.
     * Called by the drivetrain's telemetry callback (e.g. each odometry update).
     */
    public void telemeterize(SwerveDriveState state) {
        /* MegaTag2: send robot yaw to Limelight and apply vision pose when quality is good */
        megaTag.update();

        /* Robot: per-module data (units in key names) */
        for (int i = 0; i < 4; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> mod = drivetrain.getModule(i);
            TalonFX steer = mod.getSteerMotor();
            TalonFX drive = mod.getDriveMotor();
            CANcoder cancoder = mod.getEncoder();

            // Steering motor angle: rotations -> degrees
            modSteeringAngle_deg[i].set(steer.getPosition().getValueAsDouble() * ROTATIONS_TO_DEGREES);
            // CANcoder angle: rotations -> degrees
            modCANcoderAngle_deg[i].set(cancoder.getAbsolutePosition().getValueAsDouble() * ROTATIONS_TO_DEGREES);
            // Drive motor velocity: already in rot/s (RPS)
            modDriveVelocity_rps[i].set(drive.getVelocity().getValueAsDouble());
            // Supply voltage to the module (from drive motor, V)
            modSupplyVoltage_V[i].set(drive.getSupplyVoltage().getValueAsDouble());
        }

        /* Limelight: tid (dimensionless), tx/ty (deg), ta (0–100 %) */
        limelightTid.set(Limelight.getTid());
        limelightTx_deg.set(Limelight.tx());
        limelightTy_deg.set(Limelight.ty());
        limelightTa_percent.set(Limelight.ta());

        /* Shooter: speed (RPS), supply voltage (V) */
        shooterSpeed_rps.set(shooter.getShooterVelocity());
        shooterSupplyVoltage_V.set(shooter.getShooterSupplyVoltage());
    }
}
