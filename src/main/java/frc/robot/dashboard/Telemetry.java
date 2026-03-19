package frc.robot.dashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;

import frc.robot.assist.DriverAssist;
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

    /* Robot pose for AdvantageScope field view */
    private final StructPublisher<Pose2d> robotPose =
            robotTable.getStructTopic("Pose", Pose2d.struct).publish();

    /* Gyro */
    private final DoublePublisher gyroYaw_deg   = robotTable.getDoubleTopic("GyroYaw_deg").publish();
    private final DoublePublisher gyroAngle_deg  = robotTable.getDoubleTopic("GyroAngle_deg").publish();

    /* Limelight diagnostics */
    private final NetworkTable limelightTable = inst.getTable("Limelight");
    private final BooleanPublisher limelightConnected    = limelightTable.getBooleanTopic("connected").publish();
    private final DoublePublisher  limelightTv           = limelightTable.getDoubleTopic("tv").publish();
    private final DoublePublisher  limelightTid          = limelightTable.getDoubleTopic("tid").publish();
    private final DoublePublisher  limelightTx_deg       = limelightTable.getDoubleTopic("tx_deg").publish();
    private final DoublePublisher  limelightTy_deg       = limelightTable.getDoubleTopic("ty_deg").publish();
    private final DoublePublisher  limelightTa_percent   = limelightTable.getDoubleTopic("ta_percent").publish();
    private final IntegerPublisher limelightTagCount     = limelightTable.getIntegerTopic("rawTagCount").publish();
    private final StringPublisher  limelightTagIds       = limelightTable.getStringTopic("visibleTagIds").publish();
    private final BooleanPublisher limelightHasAllianceTarget = limelightTable.getBooleanTopic("hasAllianceTarget").publish();

    /* AimAssist PID diagnostics (camera TX-based alignment) */
    private final NetworkTable aimTable          = inst.getTable("AimAssist");
    private final BooleanPublisher aimActive     = aimTable.getBooleanTopic("active").publish();
    private final BooleanPublisher aimAimed      = aimTable.getBooleanTopic("isAimed").publish();
    private final DoublePublisher aimCurrentTx_deg  = aimTable.getDoubleTopic("currentTx_deg").publish();
    private final DoublePublisher aimTargetTx_deg   = aimTable.getDoubleTopic("targetTx_deg").publish();
    private final DoublePublisher aimTxError_deg    = aimTable.getDoubleTopic("txError_deg").publish();
    private final DoublePublisher aimOutput_radps   = aimTable.getDoubleTopic("output_radps").publish();
    private final DoublePublisher aimDistance_m     = aimTable.getDoubleTopic("distance_m").publish();
    private final DoublePublisher aimLockedTagId    = aimTable.getDoubleTopic("lockedTagId").publish();

    /* Vision: raw MegaTag pose (camera-only, not fused with odometry) */
    private final NetworkTable visionTable            = inst.getTable("Vision");
    private final StructPublisher<Pose2d> megaTagPose =
            visionTable.getStructTopic("MegaTagPose", Pose2d.struct).publish();
    private final BooleanPublisher megaTagHasPose     = visionTable.getBooleanTopic("hasMegaTagPose").publish();

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

        /* Robot pose (for AdvantageScope field view) */
        robotPose.set(state.Pose);

        /* Gyro */
        gyroYaw_deg.set(state.Pose.getRotation().getDegrees());
        gyroAngle_deg.set(state.RawHeading.getDegrees());

        /* Limelight diagnostics */
        limelightConnected.set(Limelight.isConnected());
        limelightTv.set(Limelight.hasTarget() ? 1.0 : 0.0);
        limelightTid.set(Limelight.getTid());
        limelightTx_deg.set(Limelight.tx());
        limelightTy_deg.set(Limelight.ty());
        limelightTa_percent.set(Limelight.ta());
        limelightHasAllianceTarget.set(driverAssist.hasAnyAllianceTarget());

        /* AimAssist diagnostics (camera TX-based alignment) */
        aimActive.set(driverAssist.isAimActive());
        aimAimed.set(driverAssist.isAimed());
        aimCurrentTx_deg.set(driverAssist.getLastTxDeg());
        aimTargetTx_deg.set(0.0);
        aimTxError_deg.set(driverAssist.getLastHeadingErrorDeg());
        aimOutput_radps.set(driverAssist.getLastAimOutput());
        aimDistance_m.set(driverAssist.getLastDistanceM());
        aimLockedTagId.set(driverAssist.getLockedTagId());

        /* Vision: raw MegaTag pose from camera */
        Pose2d accepted = megaTag.getLastAcceptedPose();
        megaTagHasPose.set(accepted != null);
        if (accepted != null) megaTagPose.set(accepted);

        // rawfiducials: count detected tags and build comma-separated ID string
        double[] raw = Limelight.getRawFiducials();
        int tagCount = raw.length / 7;
        limelightTagCount.set(tagCount);
        if (tagCount == 0) {
            limelightTagIds.set("none");
        } else {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i + 7 <= raw.length; i += 7) {
                if (sb.length() > 0) sb.append(',');
                sb.append((int) raw[i]);
            }
            limelightTagIds.set(sb.toString());
        }
    }

}
