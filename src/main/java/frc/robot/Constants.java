package frc.robot;

import static edu.wpi.first.units.Units.*;

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

    // ---------------- CONTROLLER CONSTANTS ----------------
    public static final int CONTROLLER_PORT = 0;

    public static final int LEFT_BUTTON = 5;

    public static final int RIGHT_BUTTON = 6;

    // ---------------- LIMELIGHT CONSTANTS ----------------
    public static final int LL_AIM_PIPELINE = 0;

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

    /** Shooter subsystem: TA → RPS bands and velocity PID (Kraken, rotations/sec). */
    public static final class Shooter {
        private Shooter() {}

        // ---------------- TA → TARGET RPS (velocity control) ----------------
        // Bands: smaller TA = farther = higher speed. Tune RPS on robot to match shot consistency.
        /** TA >= 0.6 (0.6–0.8%): close */
        public static final double TA_BAND_CLOSE_LO = 0.6;
        public static final double SHOOTER_RPS_TA_06_08 = 25.0;
        /** 0.4 <= TA < 0.6 */
        public static final double TA_BAND_MID_HI_LO = 0.4;
        public static final double SHOOTER_RPS_TA_04_06 = 35.0;
        /** 0.2 <= TA < 0.4 */
        public static final double TA_BAND_FAR_LO = 0.2;
        public static final double SHOOTER_RPS_TA_02_04 = 45.0;
        /** TA < 0.2: farthest */
        public static final double SHOOTER_RPS_TA_00_02 = 55.0;

        // ---------------- VELOCITY PID (Kraken) ----------------
        public static final double SHOOTER_VEL_KP = 0.15;
        public static final double SHOOTER_VEL_KI = 0.0;
        public static final double SHOOTER_VEL_KD = 0.0;
        public static final double SHOOTER_VEL_KV = 0.12;   // ~1 / (free speed per volt); tune to hold under load
        public static final double SHOOTER_VEL_KS = 0.0;
    }
}
