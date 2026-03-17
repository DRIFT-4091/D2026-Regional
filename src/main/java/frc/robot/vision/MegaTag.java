package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * MegaTag1 vision pose integration for Limelight.
 * Reads botpose_wpiblue every cycle and pushes valid measurements to the
 * drivetrain pose estimator. No robot yaw feed required.
 * Poses are rejected on high latency, too few tags, fast spin, or out-of-field
 * position so odometry and aim/shooter behavior are never corrupted by bad vision.
 */
public class MegaTag {
    /** botpose_wpiblue: [x, y, z, roll, pitch, yaw, latency_ms, tagCount, tagSpan, avgTagDist, ...] */
    private static final int INDEX_X = 0;
    private static final int INDEX_Y = 1;
    private static final int INDEX_YAW = 5;
    private static final int INDEX_LATENCY_MS = 6;
    private static final int INDEX_TAG_COUNT = 7;
    private static final int INDEX_AVG_TAG_DIST = 9;
    private static final int MIN_ARRAY_LENGTH = 8;

    /** FRC 2026 REBUILT field bounds (m) with small margin for robot at perimeter. */
    private static final double FIELD_X_MIN = -0.5;
    private static final double FIELD_X_MAX = 17.0;
    private static final double FIELD_Y_MIN = -0.5;
    private static final double FIELD_Y_MAX = 8.7;

    private final CommandSwerveDrivetrain drivetrain;
    private final NetworkTable table;
    private boolean lastUpdateWasGood;

    public MegaTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.table = NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_TABLE_NAME);
        this.lastUpdateWasGood = false;
    }

    /**
     * Returns true if the given vision pose metadata passes quality checks (for unit testing / reuse).
     */
    static boolean isVisionPoseAcceptable(int tagCount, double latencyMs, double omegaDegS) {
        if (tagCount < Constants.VISION_MIN_TAG_COUNT) return false;
        if (latencyMs > Constants.VISION_MAX_LATENCY_MS) return false;
        if (Math.abs(omegaDegS) > Constants.VISION_MAX_ANGULAR_VELOCITY_DEG_S) return false;
        return true;
    }

    /**
     * Call every cycle (e.g. from drivetrain telemetry or robot periodic).
     * Reads MegaTag1 pose and adds a vision measurement only when the pose
     * passes quality checks.
     */
    public void update() {
        double yawRateDegS = Math.toDegrees(drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond);

        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        lastUpdateWasGood = false;

        if (botpose.length < MIN_ARRAY_LENGTH) {
            return;
        }

        double latencyMs = botpose[INDEX_LATENCY_MS];
        int tagCount = (int) botpose[INDEX_TAG_COUNT];

        if (!isVisionPoseAcceptable(tagCount, latencyMs, yawRateDegS)) {
            return;
        }

        double x = botpose[INDEX_X];
        double y = botpose[INDEX_Y];
        if (x < FIELD_X_MIN || x > FIELD_X_MAX || y < FIELD_Y_MIN || y > FIELD_Y_MAX) {
            return; // pose off field, reject to avoid corrupting estimator
        }

        double timestampSeconds = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
        Pose2d visionPose = new Pose2d(
                x,
                y,
                Rotation2d.fromDegrees(botpose[INDEX_YAW]));

        double avgDistance = botpose.length > INDEX_AVG_TAG_DIST ? botpose[INDEX_AVG_TAG_DIST] : 1.0;
        double distanceFactor = Math.max(1.0, avgDistance * avgDistance);

        Matrix<N3, N1> stdDevs = tagCount >= 2
                ? VecBuilder.fill(
                        Constants.VISION_STD_DEV_XY_GOOD * distanceFactor,
                        Constants.VISION_STD_DEV_XY_GOOD * distanceFactor,
                        Constants.VISION_STD_DEV_THETA_GOOD_RAD * distanceFactor)
                : VecBuilder.fill(
                        Constants.VISION_STD_DEV_XY_SINGLE_TAG * distanceFactor,
                        Constants.VISION_STD_DEV_XY_SINGLE_TAG * distanceFactor,
                        Constants.VISION_STD_DEV_THETA_SINGLE_TAG_RAD * distanceFactor);

        drivetrain.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
        lastUpdateWasGood = true;
    }

    /** True if the last update applied a vision measurement (pose was trusted). */
    public boolean hasGoodVisionPose() {
        return lastUpdateWasGood;
    }
}
