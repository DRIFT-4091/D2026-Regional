package frc.robot.assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants;
import frc.robot.vision.Limelight;

/**
 * Handles driver assist features including camera-based auto-alignment and shooter assist.
 *
 * The driver selects the alignment target type (Hub / Trench / Human Player) on the Elastic
 * Dashboard before pressing RB. On RB press, the best tag of the selected type is locked:
 *   - Hub: ambiguous pairs resolved by smallest |tx|; then closest by ta among the rest.
 *   - Trench / Human Player: closest tag by ta.
 *
 * A heading PID then drives the locked tag's TX to zero. No field pose or odometry involved.
 */
public class DriverAssist {

    private final SendableChooser<AlignTarget> alignTargetChooser;
    private final PIDController aimPid;

    // ---- Alignment suppression state ----
    /** True once the robot has reached the setpoint; output suppressed until robot drifts back out. */
    private boolean alignmentComplete = false;
    /** True if alignment timed out before converging; stays suppressed until resetAimPid(). */
    private boolean alignmentTimedOut = false;
    /** FPGA timestamp (s) when this alignment session started. */
    private double alignmentStartTimeS = -1;

    // ---- Locked tag target (set at RB press, held for the full session) ----
    /** Tag ID locked at RB press. -1 = no lock. */
    private int lockedTagId = -1;

    // ---- Telemetry (last computed values) ----
    private double lastTxDeg           = 0.0;
    private double lastHeadingErrorDeg = 0.0;
    private double lastAimOutput       = 0.0;
    private double lastDistanceM       = 0.0;
    private boolean aimActive          = false;

    public double  getLastTxDeg()           { return lastTxDeg; }
    public double  getLastHeadingErrorDeg() { return lastHeadingErrorDeg; }
    public double  getLastAimOutput()       { return lastAimOutput; }
    public double  getLastDistanceM()       { return lastDistanceM; }
    public boolean isAimActive()            { return aimActive; }
    public int     getLockedTagId()         { return lockedTagId; }

    public DriverAssist(SendableChooser<AlignTarget> alignTargetChooser) {
        this.alignTargetChooser = alignTargetChooser;
        aimPid = new PIDController(Constants.AIM_KP, Constants.AIM_KI, Constants.AIM_KD);
        aimPid.setTolerance(Constants.AIM_TOLERANCE);
        // TX is bounded [-29.8, 29.8] — no continuous input wrap needed
        aimPid.setIntegratorRange(-Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
    }

    /** Returns the tag-ID array for the currently selected alignment target. */
    private int[] getActiveTagIds() {
        AlignTarget target = alignTargetChooser.getSelected();
        if (target == null) return Constants.HUB_TAG_IDS;
        switch (target) {
            case TRENCH:       return Constants.TRENCH_TAG_IDS;
            case HUMAN_PLAYER: return Constants.HUMAN_PLAYER_TAG_IDS;
            default:           return Constants.HUB_TAG_IDS;
        }
    }

    /**
     * Returns true if any tag matching the currently selected alignment target is visible.
     * Works in MegaTag2 mode (uses rawfiducials, not tid).
     */
    public boolean hasAnyAllianceTarget() {
        return Limelight.hasAnyTag(getActiveTagIds());
    }

    /**
     * Target shooter velocity (RPS) from the closest visible hub tag's TA.
     * Always uses hub tags regardless of alignment target — shooting is always at the hub.
     * Returns 0 if no hub tag is visible.
     */
    public double getShooterTargetRPSFromLimelight() {
        if (!Limelight.hasAnyTag(Constants.HUB_TAG_IDS)) return 0.0;
        return Constants.Shooter.getShooterTargetRPSFromTA(Limelight.ta());
    }

    /**
     * Resets the aim PID and clears all alignment suppression state.
     * Locks the best tag for the currently selected alignment target:
     *   - Hub: ambiguous pairs resolved by |tx|, then closest by ta.
     *   - Trench / Human Player: closest by ta.
     * Call when RB is pressed to start a fresh alignment session.
     */
    public void resetAimPid() {
        aimPid.reset();
        alignmentComplete   = false;
        alignmentTimedOut   = false;
        alignmentStartTimeS = -1;

        AlignTarget target = alignTargetChooser.getSelected();
        if (target == null) target = AlignTarget.HUB;

        if (target == AlignTarget.HUB) {
            lockedTagId = Limelight.getBestHubTagId(
                    Constants.HUB_TAG_IDS, Constants.HUB_AMBIGUOUS_PAIRS);
        } else {
            lockedTagId = Limelight.getBestTagId(getActiveTagIds());
        }
    }

    /**
     * Returns true when the heading PID is within tolerance (robot is aimed at target).
     */
    public boolean isAimed() {
        return aimPid.atSetpoint();
    }

    /**
     * Computes the rotation rate (rad/s) needed to center the locked tag in the camera view.
     *
     * PIDs the Limelight TX of the locked tag to zero. No field pose or odometry used.
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

        // Only align when a tag was locked at RB press and is currently visible.
        if (lockedTagId == -1 || !Limelight.hasTarget(lockedTagId)) {
            aimActive = false;
            return 0.0;
        }

        double[] txTy = Limelight.getTagTxTy(lockedTagId);
        if (txTy == null) {
            aimActive = false;
            return 0.0;
        }
        double tx = txTy[0];

        // PID: drives TX to 0. calculate(measurement=tx, setpoint=0) yields -Kp*tx.
        // This drivetrain uses positive rotationalRate = CW, so we negate:
        // if tx > 0 (tag right of center), output becomes positive → CW rotation → correct.
        double output = -aimPid.calculate(tx, 0.0);

        // Distance: read distToRobot from rawfiducials for the locked tag.
        double dist = 0.0;
        double[] raw = Limelight.getRawFiducials();
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            if ((int) raw[i] == lockedTagId) { dist = raw[i + 5]; break; }
        }

        lastTxDeg           = tx;
        lastHeadingErrorDeg = tx; // error IS tx (setpoint is 0)
        lastAimOutput       = output;
        lastDistanceM       = dist;

        // Lock complete once on-target.
        if (aimPid.atSetpoint()) {
            alignmentComplete = true;
        }

        // Auto-resume: robot drifted past threshold → restart correction.
        if (alignmentComplete && Math.abs(tx) > Constants.AIM_RESUME_THRESHOLD_DEG) {
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
        return MathUtil.clamp(output, -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
    }

    /** Calculates aim correction with no caller-supplied omega. */
    public double calculateAimCorrection() {
        return calculateAimCorrection(0.0);
    }
}
