package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.Limelight;
import frc.robot.vision.MegaTag;

/**
 * Publishes telemetry to NetworkTables: Robot (per-module data) and Limelight.
 * Also runs MegaTag2 vision pose updates (with fallback when pose is bad).
 * SysId widgets are in SysIdDashboard. Units are in the key names where applicable.
 */
public class Telemetry {
    private final CommandSwerveDrivetrain drivetrain;
    private final MegaTag megaTag;
    private final DriverAssist driverAssist;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot: per-module steering angle (deg), CANcoder angle (deg), drive velocity (rps), supply voltage (V) */
    private final NetworkTable robotTable = inst.getTable("Robot");
    private final DoublePublisher[] modSteeringAngle_deg = new DoublePublisher[4];
    private final DoublePublisher[] modCANcoderAngle_deg = new DoublePublisher[4];
    private final DoublePublisher[] modDriveVelocity_rps = new DoublePublisher[4];
    private final DoublePublisher[] modSupplyVoltage_V = new DoublePublisher[4];

    /* Robot: acceleration (m/s²) derived from successive ChassisSpeeds */
    private final DoublePublisher accelX_mps2 = robotTable.getDoubleTopic("AccelX_mps2").publish();
    private final DoublePublisher accelY_mps2 = robotTable.getDoubleTopic("AccelY_mps2").publish();
    private final DoublePublisher accelMag_mps2 = robotTable.getDoubleTopic("AccelMag_mps2").publish();

    private double prevVx = 0.0;
    private double prevVy = 0.0;
    private double prevTimestamp = -1.0;

    /* Limelight: tid (dimensionless), tx/ty (deg), ta (%), hasAnyAllianceTarget */
    private final NetworkTable limelightTable = inst.getTable("Limelight");
    private final DoublePublisher limelightTid = limelightTable.getDoubleTopic("tid").publish();
    private final DoublePublisher limelightTx_deg = limelightTable.getDoubleTopic("tx_deg").publish();
    private final DoublePublisher limelightTy_deg = limelightTable.getDoubleTopic("ty_deg").publish();
    private final DoublePublisher limelightTa_percent = limelightTable.getDoubleTopic("ta_percent").publish();
    private final BooleanPublisher limelightHasAllianceTarget = limelightTable.getBooleanTopic("hasAllianceTarget").publish();

    private static final double ROTATIONS_TO_DEGREES = 360.0;

    /** Returns angle in degrees wrapped to [0, 360). */
    private static double mod360(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    /**
     * Constructs telemetry with references to drivetrain and MegaTag (vision updates).
     */
    public Telemetry(CommandSwerveDrivetrain drivetrain, MegaTag megaTag, DriverAssist driverAssist) {
        this.drivetrain = drivetrain;
        this.megaTag = megaTag;
        this.driverAssist = driverAssist;

        for (int i = 0; i < 4; i++) {
            NetworkTable modTable = robotTable.getSubTable("Module" + i);
            modSteeringAngle_deg[i] = modTable.getDoubleTopic("SteeringAngle_deg").publish();
            modCANcoderAngle_deg[i] = modTable.getDoubleTopic("CANcoderAngle_deg").publish();
            modDriveVelocity_rps[i] = modTable.getDoubleTopic("DriveVelocity_rps").publish();
            modSupplyVoltage_V[i] = modTable.getDoubleTopic("SupplyVoltage_V").publish();
        }
    }

    /**
     * Publishes Robot (module) and Limelight data to NetworkTables and runs MegaTag2 vision updates.
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

            // Steering motor angle: rotations -> degrees, mod 360 for Shuffleboard
            modSteeringAngle_deg[i].set(mod360(steer.getPosition().getValueAsDouble() * ROTATIONS_TO_DEGREES));
            // CANcoder angle: rotations -> degrees, mod 360 for Shuffleboard
            modCANcoderAngle_deg[i].set(mod360(cancoder.getAbsolutePosition().getValueAsDouble() * ROTATIONS_TO_DEGREES));
            // Drive motor velocity: already in rot/s (RPS)
            modDriveVelocity_rps[i].set(drive.getVelocity().getValueAsDouble());
            // Supply voltage to the module (from drive motor, V)
            modSupplyVoltage_V[i].set(drive.getSupplyVoltage().getValueAsDouble());
        }

        /* Robot: acceleration (field-relative X/Y and magnitude, m/s²) */
        double t = state.Timestamp;
        double vx = state.Speeds.vxMetersPerSecond;
        double vy = state.Speeds.vyMetersPerSecond;
        if (prevTimestamp > 0.0) {
            double dt = t - prevTimestamp;
            if (dt > 0.0) {
                double ax = (vx - prevVx) / dt;
                double ay = (vy - prevVy) / dt;
                accelX_mps2.set(ax);
                accelY_mps2.set(ay);
                accelMag_mps2.set(Math.hypot(ax, ay));
            }
        }
        prevVx = vx;
        prevVy = vy;
        prevTimestamp = t;

        /* Limelight: tid (dimensionless), tx/ty (deg), ta (0–100 %), hasAllianceTarget */
        limelightTid.set(Limelight.getTid());
        limelightTx_deg.set(Limelight.tx());
        limelightTy_deg.set(Limelight.ty());
        limelightTa_percent.set(Limelight.ta());
        limelightHasAllianceTarget.set(driverAssist.hasAnyAllianceTarget());
    }
}
