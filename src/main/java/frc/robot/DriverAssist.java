package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.vision.Limelight;

/**
 * Handles driver assist features including Limelight-based aim assist.
 */
public class DriverAssist {
    /** Pre-computed combined tag IDs when alliance is unknown (avoids per-cycle allocation). */
    private static final int[] BOTH_HUB_TAG_IDS;
    static {
        BOTH_HUB_TAG_IDS = new int[Constants.BLUE_HUB_TAG_IDS.length + Constants.RED_HUB_TAG_IDS.length];
        System.arraycopy(Constants.BLUE_HUB_TAG_IDS, 0, BOTH_HUB_TAG_IDS, 0, Constants.BLUE_HUB_TAG_IDS.length);
        System.arraycopy(Constants.RED_HUB_TAG_IDS, 0, BOTH_HUB_TAG_IDS, Constants.BLUE_HUB_TAG_IDS.length, Constants.RED_HUB_TAG_IDS.length);
    }

    private final PIDController aimPid;

    public DriverAssist() {
        aimPid = new PIDController(Constants.AIM_KP, Constants.AIM_KI, Constants.AIM_KD);
        aimPid.setTolerance(Constants.AIM_TOLERANCE);
        aimPid.enableContinuousInput(-180, 180);
        aimPid.setIntegratorRange(-Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
    }

    /**
     * Gets the AprilTag IDs for the current alliance.
     * When alliance is unknown (e.g. before match), returns both blue and red IDs so aim/shooter still work.
     */
    public int[] getAllianceTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return BOTH_HUB_TAG_IDS;
        }
        return alliance.get() == DriverStation.Alliance.Blue
            ? Constants.BLUE_HUB_TAG_IDS
            : Constants.RED_HUB_TAG_IDS;
    }

    /**
     * Checks if any alliance-specific AprilTag is visible.
     * Reads tv and tid once (2 NT reads total) instead of per-tag.
     */
    public boolean hasAnyAllianceTarget() {
        if (!Limelight.hasTarget()) return false;
        int tid = (int) Limelight.getTid();
        for (int tagId : getAllianceTagIds()) {
            if (tagId == tid) return true;
        }
        return false;
    }

    /**
     * Target shooter velocity (rotations per second) from Limelight TA.
     * Uses continuous interpolation (InterpolatingDoubleTreeMap) for smoother shots across distance.
     * Returns 0 if no AprilTag (shooter should stop).
     */
    public double getShooterTargetRPSFromLimelight() {
        if (!hasAnyAllianceTarget()) {
            return 0.0;
        }
        return Constants.Shooter.getShooterTargetRPSFromTA(Limelight.ta());
    }

    /**
     * Resets the aim PID controller.
     */
    public void resetAimPid() {
        aimPid.reset();
    }

    /**
     * Calculates the aim correction based on Limelight data (tx = horizontal offset to target).
     * Uses pipeline + capture latency to predict current tx and reduce overshoot at high rotation speeds.
     *
     * @param omegaRadPerSec Current robot angular velocity (rad/s); use 0 if unknown
     * @return Rotation rate in rad/s to drive tx toward 0
     */
    public double calculateAimCorrection(double omegaRadPerSec) {
        double tx = Limelight.tx();
        double totalLatencyMs = Limelight.tl() + Limelight.cl();
        double omegaDegS = Math.toDegrees(omegaRadPerSec);
        double predictedTx = tx - omegaDegS * totalLatencyMs / 1000.0;
        return aimPid.calculate(predictedTx, 0.0);
    }

    /**
     * Calculates the aim correction without latency compensation (omega assumed 0).
     * @return Rotation rate in rad/s to drive tx toward 0
     */
    public double calculateAimCorrection() {
        return calculateAimCorrection(0.0);
    }
}
