package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.generated.TunerConstants;

/**
 * Central location for all robot constants.
 */
public final class Constants {
    private Constants() {}

    // ---------------- DRIVETRAIN CONSTANTS ----------------
    public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Drivetrain deadbands
    public static final double DRIVE_DEADBAND = MAX_SPEED * 0.2;
    public static final double ROTATION_DEADBAND = MAX_ANGULAR_RATE * 0.2;

    /** Timeout (s) for field-centric seed / point wheels forward when enabling. */
    public static final double FIELD_CENTRIC_SEED_TIMEOUT_S = 0.75;

    // ---------------- CONTROLLER CONSTANTS ----------------
    public static final int CONTROLLER_PORT = 0;

    public static final int LEFT_BUTTON = 5;

    public static final int RIGHT_BUTTON = 6;

    // ---------------- LIMELIGHT CONSTANTS ----------------
    /** NetworkTable table name for the Limelight (change if camera has a different hostname). */
    public static final String LIMELIGHT_TABLE_NAME = "limelight";
    public static final int LL_AIM_PIPELINE = 0;

    // ---------------- MEGATAG2 / VISION POSE CONSTANTS ----------------
    /** Max vision pipeline latency (ms) to accept a pose. Reject older measurements. */
    public static final double VISION_MAX_LATENCY_MS = 100.0;
    /** Min AprilTag count to accept pose (1 = single tag ok; 2 = prefer multi-tag). */
    public static final int VISION_MIN_TAG_COUNT = 1;
    /** Reject vision updates when |angular velocity| exceeds this (deg/s). Reduces blur/ambiguity. */
    public static final double VISION_MAX_ANGULAR_VELOCITY_DEG_S = 360.0;
    /** Std devs [x, y, theta] when pose is trusted (multi-tag or good single-tag). Meters, meters, radians. */
    public static final double VISION_STD_DEV_XY_GOOD = 0.5;
    public static final double VISION_STD_DEV_THETA_GOOD_RAD = Math.toRadians(6.0);
    /** Std devs when only one tag visible (looser, especially theta). */
    public static final double VISION_STD_DEV_XY_SINGLE_TAG = 0.7;
    public static final double VISION_STD_DEV_THETA_SINGLE_TAG_RAD = Math.toRadians(15.0);

    // ---------------- APRILTAG CONSTANTS ----------------
    // REBUILT FRC Challenge AprilTag IDs
    public static final int[] BLUE_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] RED_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

    // ---------------- AIM ASSIST PID CONSTANTS ----------------
    // For 10–12 ball bursts: small KI helps correct steady drift while holding on target.
    public static final double AIM_KP = 0.06;
    public static final double AIM_KI = 0.008;   // was 0; slight integral helps sustained aim during long bursts
    public static final double AIM_KD = 0.001;
    public static final double AIM_TOLERANCE = 1.0;

    /** Max rotation speed for aim assist */
    public static final double MAX_AIM_RAD_PER_SEC = 2.5;

    /** Shooter subsystem: TA → RPS (interpolated) and velocity PID (Kraken, rotations/sec). */
    public static final class Shooter {
        private Shooter() {}

        // ---------------- TA → TARGET RPS (continuous interpolation) ----------------
        // Smaller TA = farther = higher RPS. InterpolatingDoubleTreeMap gives smooth shots across distance.
        // TA is 0–100 (percentage of image area) as returned by Limelight.
        private static final InterpolatingDoubleTreeMap TA_TO_RPS = new InterpolatingDoubleTreeMap();
        static {
            TA_TO_RPS.put(0.0,  55.0);
            TA_TO_RPS.put(20.0, 45.0);
            TA_TO_RPS.put(40.0, 35.0);
            TA_TO_RPS.put(60.0, 25.0);
            TA_TO_RPS.put(100.0, 25.0);  // cap close range
        }

        /** Target shooter RPS from Limelight TA (0–100%). Clamps TA to [0, 100]. Returns 0 if no target. */
        public static double getShooterTargetRPSFromTA(double ta) {
            if (ta <= 0) return 0.0;
            double clamped = Math.max(0.0, Math.min(100.0, ta));
            return TA_TO_RPS.get(clamped);
        }

        // ---------------- VELOCITY PID (Kraken) ----------------
        public static final double SHOOTER_VEL_KP = 0.15;
        public static final double SHOOTER_VEL_KI = 0.0;
        public static final double SHOOTER_VEL_KD = 0.0;
        public static final double SHOOTER_VEL_KV = 0.12;   // ~1 / (free speed per volt); tune to hold under load
        public static final double SHOOTER_VEL_KS = 0.0;

        /** RPS tolerance for "at target" rumble (rotations per second). */
        public static final double SHOOTER_RPS_RUMBLE_TOLERANCE = 2.0;
        /** Joystick rumble strength when shooter is at target (0–1). */
        public static final double SHOOTER_RUMBLE_STRENGTH = 0.5;
    }
}
