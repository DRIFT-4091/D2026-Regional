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
    public static final double DRIVE_DEADBAND = MAX_SPEED * 0.15;
    public static final double ROTATION_DEADBAND = MAX_ANGULAR_RATE * 0.15;

    /** Timeout (s) for field-centric seed / point wheels forward when enabling. */
    public static final double FIELD_CENTRIC_SEED_TIMEOUT_S = 0.75;

    /**
     * Offset (degrees) added to robot heading when pointing wheels "forward".
     * Use this when all four wheels look tilted the same way (e.g. all slightly left).
     * If wheels look tilted left when the robot is straight, try -2 or -3; if right, try +2 or +3.
     * For per-wheel tilt (one wheel off from the others), tune encoder offsets in
     * TunerConstants (kFrontLeftEncoderOffset, kFrontRightEncoderOffset, etc.) or re-run
     * Phoenix Tuner / steer calibration; those define each module's mechanical zero.
     */
    public static final double WHEEL_FORWARD_OFFSET_DEG = 0;

    /**
     * When pointing wheels forward, we constrain steer angle to this semicircle (in rotations)
     * so bevel gears always face inward. 0.5 = use [0, 0.5) rotations; if your bevels face
     * inward on the other half, set to 0.0 to use [0.5, 1.0) instead.
     */
    public static final double BEVEL_IN_SEMICIRCLE_MAX_ROTATIONS = 0.5;

    /**
     * Module indices (0=FL, 1=FR, 2=BL, 3=BR) that use semicircle [0.5, 1.0) for bevel-in.
     * 3 = Back Right (use other semicircle so bevel gear faces inside).
     */
    public static final int[] BEVEL_IN_OTHER_SEMICIRCLE_MODULES = { 3 };

    /** Basket wobble (X button): forward/back speed (m/s) and oscillation frequency (Hz). */
    public static final double WOBBLE_SPEED_MPS = 1.2;
    public static final double WOBBLE_HZ = 4.0;

    // ---------------- CONTROLLER CONSTANTS ----------------
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    /** Raw axis deadband (0–1) so small stick movement doesn't send velocity; larger value helps wheels return to forward when stick is released. */
    public static final double JOYSTICK_DEADBAND = 0.12;
    /** Axis index for rotation (4 = Xbox right X). If rotation doesn't work on GameSir, try 2 or 3. */
    public static final int ROTATION_AXIS = 4;

    public static final int LEFT_BUTTON = 5;

    public static final int RIGHT_BUTTON = 6;

    /** Face button for basket wobble (X on Xbox layout). If X doesn't work on GameSir, try 2 or 4. */
    public static final int X_BUTTON = 3;

    // ---------------- LIMELIGHT CONSTANTS ----------------
    /** NetworkTable table name for the Limelight (change if camera has a different hostname). */
    public static final String LIMELIGHT_TABLE_NAME = "limelight";
    public static final int LL_AIM_PIPELINE = 0;

    /** Limelight mount: angle down from horizontal (degrees). Used by MegaTag2 orientation feed. */
    public static final double LL_MOUNT_ANGLE_DEG = 25.0;

    // ---------------- MEGATAG2 / VISION POSE CONSTANTS ----------------
    /** Max vision pipeline latency (ms) to accept a pose. Reject older measurements. */
    public static final double VISION_MAX_LATENCY_MS = 100.0;
    /** Min AprilTag count to accept pose (1 = single tag ok; 2 = prefer multi-tag). */
    public static final int VISION_MIN_TAG_COUNT = 1;
    /** Reject vision updates when |angular velocity| exceeds this (deg/s). Reduces blur/ambiguity. */
    public static final double VISION_MAX_ANGULAR_VELOCITY_DEG_S = 360.0;
    /** Std devs [x, y, theta] when pose is trusted (multi-tag or good single-tag). Meters, meters, radians. */
    public static final double VISION_STD_DEV_XY_GOOD = 0.5;
    /** MegaTag2 reflects the heading you feed it back — never let vision override the gyro heading. */
    public static final double VISION_STD_DEV_THETA_GOOD_RAD = 9999.0;
    /** Std devs when only one tag visible (looser, especially theta). */
    public static final double VISION_STD_DEV_XY_SINGLE_TAG = 0.7;
    public static final double VISION_STD_DEV_THETA_SINGLE_TAG_RAD = 9999.0;

    // ---------------- HUB CENTER COORDINATES (field-relative, WPILib blue origin, meters) ----------------
    // Used by pose-based auto-alignment: robot turns to face atan2(hubY - robotY, hubX - robotX).
    // Verify these against your 2026-rebuilt-andymark.json field layout file.
    public static final double BLUE_HUB_CENTER_X = 4.62534;
    public static final double BLUE_HUB_CENTER_Y = 4.034536;
    public static final double RED_HUB_CENTER_X  = 11.915;
    public static final double RED_HUB_CENTER_Y  = 4.034536;

    // ---------------- APRILTAG CONSTANTS ----------------
    // 2026 REBUILT AprilTag IDs by zone
    public static final int[] HUB_TAG_IDS         = {9, 10, 11, 2, 8, 5, 18, 27, 21, 24, 26, 25, 23};
    public static final int[] HUMAN_PLAYER_TAG_IDS = {13, 14, 29, 30};
    public static final int[] TRENCH_TAG_IDS       = {6, 7, 12, 1, 17, 28, 22}; // 23 is a trench tag

    /**
     * Hub tag pairs that can be confused when both are visible at once.
     * When both tags of a pair are detected, align to the one with the smaller |tx|
     * (more centered in the camera view) rather than the closest by area.
     */
    public static final int[][] HUB_AMBIGUOUS_PAIRS = {{8, 9}, {10, 11}, {27, 26}, {24, 25}};

    // ---------------- AIM ASSIST PID CONSTANTS ----------------
    // For 10–12 ball bursts: small KI helps correct steady drift while holding on target.
    public static final double AIM_KP = 0.35;
    public static final double AIM_KI = 0.002;
    public static final double AIM_KD = 0.008;
    public static final double AIM_TOLERANCE = 1.5;

    /** Max rotation speed for aim assist */
    public static final double MAX_AIM_RAD_PER_SEC = 6.0;

    /** Seconds after tag acquisition before giving up on alignment (didn't converge). */
    public static final double AIM_TIMEOUT_S = 3.0;
    /**
     * Tx error (degrees) needed to re-enable alignment after reaching target.
     * If robot drifts more than this from the setpoint, correction resumes automatically.
     */
    public static final double AIM_RESUME_THRESHOLD_DEG = 4.0;

    /** Shooter subsystem: TA → RPS (interpolated) and velocity PID (Kraken, rotations/sec). */
    public static final class Shooter {
        private Shooter() {}

        // ---------------- TA → TARGET RPS (continuous interpolation) ----------------
        // Smaller TA = farther = higher RPS. InterpolatingDoubleTreeMap gives smooth shots across distance.
        // TA is 0–1 (fraction of image area) as returned by Limelight.
        private static final InterpolatingDoubleTreeMap TA_TO_RPS = new InterpolatingDoubleTreeMap();
        static {
            TA_TO_RPS.put(0.0,  70.0);
            TA_TO_RPS.put(0.2,  60.0);
            TA_TO_RPS.put(0.4,  50.0);
            TA_TO_RPS.put(0.7,  37.0);
            TA_TO_RPS.put(0.85,  30.0);
            TA_TO_RPS.put(1.0,  28.0);  // cap close range
        }

        /** Target shooter RPS from Limelight TA (0–1). Clamps TA to [0, 1]. Returns 0 if no target. */
        public static double getShooterTargetRPSFromTA(double ta) {
            if (ta <= 0) return 0.0;
            double clamped = Math.max(0.0, Math.min(1.0, ta));
            return TA_TO_RPS.get(clamped);
        }

        // ---------------- VELOCITY PID (Kraken) ----------------
        // Values from SysId characterization (2026 regional)
        public static final double SHOOTER_VEL_KP = 0.13868;
        public static final double SHOOTER_VEL_KI = 0.0;
        public static final double SHOOTER_VEL_KD = 0.003;
        public static final double SHOOTER_VEL_KV = 0.11925;
        public static final double SHOOTER_VEL_KS = 0.12745;
        public static final double SHOOTER_VEL_KA = 0.010366;

        /** Fallback shooter speed (RPS) when no AprilTag target is visible. */
        public static final double SHOOTER_DEFAULT_RPS = 60.0;
        /** RPS tolerance for "at target" rumble (rotations per second). */
        public static final double SHOOTER_RPS_RUMBLE_TOLERANCE = 2.0;
        /** Joystick rumble strength when shooter is at target (0–1). */
        public static final double SHOOTER_RUMBLE_STRENGTH = 0.5;
    }
}
