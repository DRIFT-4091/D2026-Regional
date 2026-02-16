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
    public static final double DRIVE_DEADBAND = MAX_SPEED * 0.25;
    public static final double ROTATION_DEADBAND = MAX_ANGULAR_RATE * 0.25;

    // ---------------- CONTROLLER CONSTANTS ----------------
    public static final int CONTROLLER_PORT = 1;

    // ---------------- LIMELIGHT CONSTANTS ----------------
    public static final int LL_AIM_PIPELINE = 0;

    // ---------------- APRILTAG CONSTANTS ----------------
    // REBUILT FRC Challenge AprilTag IDs
    public static final int[] BLUE_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] RED_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

    // ---------------- AIM ASSIST PID CONSTANTS ----------------
    public static final double AIM_KP = 0.06;
    public static final double AIM_KI = 0.0;
    public static final double AIM_KD = 0.001;
    public static final double AIM_TOLERANCE = 1.0;

    // ---------------- DISTANCE PID CONSTANTS ----------------
    public static final double DISTANCE_KP = 0.6;
    public static final double DISTANCE_KI = 0.0;
    public static final double DISTANCE_KD = 0.0;
    public static final double DISTANCE_TOLERANCE = 0.15;

    // ---------------- ALIGNMENT THRESHOLDS ----------------
    /** Target area at optimal shooting distance */
    public static final double DESIRED_TA = 2.0;
    /** Horizontal alignment tolerance (degrees) */
    public static final double READY_TX_DEG = 1.0;
    /** Distance tolerance (target area) */
    public static final double READY_TA_TOL = 0.15;
    /** Max rotation speed for aim assist */
    public static final double MAX_AIM_RAD_PER_SEC = 2.5;
    /** Max auto forward speed */
    public static final double MAX_AUTO_FWD_MPS = 2.0;
    /** Controller rumble intensity when ready */
    public static final double READY_RUMBLE = 1.0;

    // ---------------- SHOOTER VOLTAGE MAPPING ----------------
    // Distance-to-voltage mapping breakpoints and interpolation
    public static final double TA_VERY_CLOSE = 4.0;
    public static final double TA_OPTIMAL = 2.5;
    public static final double TA_FAR = 1.5;
    public static final double TA_VERY_FAR = 0.75;

    public static final double VOLTAGE_VERY_CLOSE = 6.0;   // at ta = 4.0
    public static final double VOLTAGE_OPTIMAL = 8.0;      // at ta = 2.5
    public static final double VOLTAGE_FAR = 10.0;         // at ta = 1.5
    public static final double VOLTAGE_VERY_FAR = 11.5;    // at ta = 0.75
    public static final double VOLTAGE_MAX = 12.0;         // at ta < 0.75

    // Interpolation slopes (change in voltage per change in ta)
    public static final double SLOPE_VERY_CLOSE_TO_OPTIMAL = 2.0 / 1.5;  // ta 4.0->2.5: 6V->8V
    public static final double SLOPE_OPTIMAL_TO_FAR = 2.0 / 1.0;         // ta 2.5->1.5: 8V->10V
    public static final double SLOPE_FAR_TO_VERY_FAR = 1.5 / 0.75;      // ta 1.5->0.75: 10V->11.5V
}
