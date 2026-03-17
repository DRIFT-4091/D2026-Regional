package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.Limelight;
import frc.robot.vision.MegaTag;

/**
 * Handles driver assist features including pose-based auto-alignment and shooter assist.
 *
 * Auto-alignment uses the drivetrain's fused pose (odometry + MegaTag1 vision) to compute
 * the angle from the robot's current position to the hub center, then runs a heading PID
 * to rotate the robot to face that angle. Alignment is only active while MT1 is providing
 * a valid field-space pose.
 */
public class DriverAssist {
    /** Pre-computed combined tag IDs when alliance is unknown (avoids per-cycle allocation). */
    private static final int[] BOTH_HUB_TAG_IDS;
    static {
        BOTH_HUB_TAG_IDS = new int[Constants.BLUE_HUB_TAG_IDS.length + Constants.RED_HUB_TAG_IDS.length];
        System.arraycopy(Constants.BLUE_HUB_TAG_IDS, 0, BOTH_HUB_TAG_IDS, 0, Constants.BLUE_HUB_TAG_IDS.length);
        System.arraycopy(Constants.RED_HUB_TAG_IDS, 0, BOTH_HUB_TAG_IDS, Constants.BLUE_HUB_TAG_IDS.length, Constants.RED_HUB_TAG_IDS.length);
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final MegaTag megaTag;
    private final PIDController aimPid;

    // ---- Alignment suppression state ----
    /** True once the robot has reached the setpoint; output suppressed until robot drifts back out. */
    private boolean alignmentComplete = false;
    /** True if alignment timed out before converging; stays suppressed until resetAimPid(). */
    private boolean alignmentTimedOut = false;
    /** FPGA timestamp (s) when this alignment session started. */
    private double alignmentStartTimeS = -1;

    // ---- Locked hub target (set at RB press, held for the full session) ----
    /** Hub center X selected when RB was pressed (meters, field-relative). */
    private double targetHubX = Constants.BLUE_HUB_CENTER_X;
    /** Hub center Y selected when RB was pressed (meters, field-relative). */
    private double targetHubY = Constants.BLUE_HUB_CENTER_Y;

    // ---- Telemetry (last computed values) ----
    private double lastCurrentHeadingDeg = 0.0;
    private double lastDesiredHeadingDeg = 0.0;
    private double lastHeadingErrorDeg   = 0.0;
    private double lastAimOutput         = 0.0;
    private double lastDistanceM         = 0.0;
    private boolean aimActive            = false;

    public double getLastCurrentHeadingDeg() { return lastCurrentHeadingDeg; }
    public double getLastDesiredHeadingDeg()  { return lastDesiredHeadingDeg; }
    public double getLastHeadingErrorDeg()    { return lastHeadingErrorDeg; }
    public double getLastAimOutput()          { return lastAimOutput; }
    public double getLastDistanceM()          { return lastDistanceM; }
    public boolean isAimActive()              { return aimActive; }

    public DriverAssist(CommandSwerveDrivetrain drivetrain, MegaTag megaTag) {
        this.drivetrain = drivetrain;
        this.megaTag = megaTag;
        aimPid = new PIDController(Constants.AIM_KP, Constants.AIM_KI, Constants.AIM_KD);
        aimPid.setTolerance(Constants.AIM_TOLERANCE);
        aimPid.enableContinuousInput(-180, 180);
        aimPid.setIntegratorRange(-Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
    }

    /**
     * Gets the hub AprilTag IDs for the current alliance.
     * When alliance is unknown returns both so aim/shooter still work.
     */
    public int[] getAllianceTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return BOTH_HUB_TAG_IDS;
        return alliance.get() == DriverStation.Alliance.Blue
            ? Constants.BLUE_HUB_TAG_IDS
            : Constants.RED_HUB_TAG_IDS;
    }

    /**
     * Checks if any hub AprilTag (red or blue) is currently visible via rawfiducials.
     * Works in MegaTag2 mode (tid is unreliable there).
     */
    public boolean hasAnyAllianceTarget() {
        return Limelight.hasAnyTag(BOTH_HUB_TAG_IDS);
    }

    /**
     * Target shooter velocity (RPS) from the closest hub tag's TA.
     * Returns 0 if no hub tag is visible.
     */
    public double getShooterTargetRPSFromLimelight() {
        if (!hasAnyAllianceTarget()) return 0.0;
        return Constants.Shooter.getShooterTargetRPSFromTA(Limelight.ta());
    }

    /**
     * Resets the aim PID and clears all alignment suppression state.
     * Call when RB is pressed to start a fresh alignment session.
     */
    public void resetAimPid() {
        aimPid.reset();
        alignmentComplete   = false;
        alignmentTimedOut   = false;
        alignmentStartTimeS = -1;

        // Snapshot pose at RB press and lock in the closer hub for the full session.
        var pose = drivetrain.getPose();
        double rx = pose.getX();
        double ry = pose.getY();
        double distBlue = Math.hypot(Constants.BLUE_HUB_CENTER_X - rx, Constants.BLUE_HUB_CENTER_Y - ry);
        double distRed  = Math.hypot(Constants.RED_HUB_CENTER_X  - rx, Constants.RED_HUB_CENTER_Y  - ry);
        if (distBlue <= distRed) {
            targetHubX = Constants.BLUE_HUB_CENTER_X;
            targetHubY = Constants.BLUE_HUB_CENTER_Y;
        } else {
            targetHubX = Constants.RED_HUB_CENTER_X;
            targetHubY = Constants.RED_HUB_CENTER_Y;
        }
    }

    /**
     * Returns true when the heading PID is within tolerance (robot is aimed at hub).
     */
    public boolean isAimed() {
        return aimPid.atSetpoint();
    }

    /**
     * Computes the rotation rate (rad/s) needed to face the hub center.
     *
     * Uses the drivetrain's fused pose (odometry + MegaTag2) to get the robot's
     * current field position and heading, draws a line to the hub center, and
     * runs a heading PID on the angle error.
     *
     * Auto-stops once aligned; auto-resumes if the robot drifts past
     * AIM_RESUME_THRESHOLD_DEG. Permanently stops if AIM_TIMEOUT_S elapses
     * without convergence (requires RB re-press to reset).
     *
     * @param omegaRadPerSec Unused — kept for API compatibility with OI/Autos callers.
     * @return Rotation rate in rad/s (positive = CCW). 0 when suppressed or timed out.
     */
    public double calculateAimCorrection(double omegaRadPerSec) {
        // Start session timer on first call.
        if (alignmentStartTimeS < 0) {
            alignmentStartTimeS = Timer.getFPGATimestamp();
        }

        // Permanently stopped until RB is re-pressed.
        if (alignmentTimedOut) {
            aimActive = false;
            return 0.0;
        }
        if (Timer.getFPGATimestamp() - alignmentStartTimeS > Constants.AIM_TIMEOUT_S) {
            alignmentTimedOut = true;
            aimActive = false;
            return 0.0;
        }

        // Only align when a hub tag is visible and MT1 has a current valid field pose.
        if (!hasAnyAllianceTarget() || !megaTag.hasGoodVisionPose()) {
            aimActive = false;
            return 0.0;
        }

        // Get robot position and heading from MT1-fused pose estimator.
        Pose2d robotPose = drivetrain.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double currentHeadingDeg = robotPose.getRotation().getDegrees();

        // Use the hub locked in at RB press (set by resetAimPid).
        double hubX = targetHubX;
        double hubY = targetHubY;

        // Vector from robot to hub center → desired heading.
        double dx = hubX - robotX;
        double dy = hubY - robotY;
        double desiredHeadingDeg = MathUtil.inputModulus(Math.toDegrees(Math.atan2(dy, dx)) + 180.0, -180.0, 180.0);
        double distanceM = Math.hypot(dx, dy);

        // Heading error wrapped to [-180, 180] for logging and resume check.
        double headingErrorDeg = MathUtil.inputModulus(desiredHeadingDeg - currentHeadingDeg, -180.0, 180.0);

        // PID: enableContinuousInput handles wrap-around.
        // positive output → CCW rotation → withRotationalRate(positive) in CTRE field-centric.
        double output = aimPid.calculate(currentHeadingDeg, desiredHeadingDeg);

        lastCurrentHeadingDeg = currentHeadingDeg;
        lastDesiredHeadingDeg = desiredHeadingDeg;
        lastHeadingErrorDeg   = headingErrorDeg;
        lastAimOutput         = output;
        lastDistanceM         = distanceM;

        // Lock complete once on-target.
        if (aimPid.atSetpoint()) {
            alignmentComplete = true;
        }

        // Auto-resume: robot drifted past threshold → restart correction.
        if (alignmentComplete && Math.abs(headingErrorDeg) > Constants.AIM_RESUME_THRESHOLD_DEG) {
            alignmentComplete = false;
            aimPid.reset();
            alignmentStartTimeS = Timer.getFPGATimestamp();
        }

        // Suppress output while alignment is complete (on-target).
        if (alignmentComplete) {
            aimActive = false;
            return 0.0;
        }

        aimActive = true;
        return output;
    }

    /** Calculates aim correction with no caller-supplied omega (omega obtained internally from drivetrain). */
    public double calculateAimCorrection() {
        return calculateAimCorrection(0.0);
    }
}
