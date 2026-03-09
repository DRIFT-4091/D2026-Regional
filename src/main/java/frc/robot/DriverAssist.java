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
     * Checks if any hub AprilTag (red or blue) is visible.
     * Accepts any tag in BOTH_HUB_TAG_IDS regardless of alliance.
     */
    public boolean hasAnyAllianceTarget() {
        if (!Limelight.hasTarget()) {
            System.out.println("[DA] No target (tv=0)");
            return false;
        }
        int tid = (int) Math.round(Limelight.getTid());
        for (int tagId : BOTH_HUB_TAG_IDS) {
            if (tagId == tid) return true;
        }
        System.out.println("[DA] tid=" + tid + " not a hub tag");
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
     * Estimates distance to target (m) from Limelight ty and mount angle.
     * Uses geometry: (targetHeight - cameraHeight) / tan(mountAngle + ty).
     */
    private static double estimateDistanceToTarget(double tyDeg) {
        double angleDeg = Constants.LL_MOUNT_ANGLE_DEG + tyDeg;
        if (angleDeg <= 0.5 || angleDeg >= 89.5) {
            return Constants.LL_DISTANCE_MAX_METERS;
        }
        double angleRad = Math.toRadians(angleDeg);
        double d = (Constants.LL_TARGET_HEIGHT_METERS - Constants.LL_CAMERA_HEIGHT_METERS) / Math.tan(angleRad);
        return Math.max(Constants.LL_DISTANCE_MIN_METERS, Math.min(Constants.LL_DISTANCE_MAX_METERS, d));
    }

    /**
     * Desired tx (degrees) so that robot center (not camera) is aimed at target.
     * Camera is offset laterally; when center is on target, camera sees target at this tx.
     */
    private static double getTxSetpointForCenterAim(double distanceMeters) {
        return -Math.toDegrees(Math.atan2(Constants.LL_LATERAL_OFFSET_METERS, distanceMeters));
    }

    /**
     * Calculates the aim correction based on Limelight data (tx = horizontal offset to target).
     * Accounts for camera mount angle (20–30°) and 4 in lateral offset so the robot center aligns.
     * Uses pipeline + capture latency to predict current tx and reduce overshoot at high rotation speeds.
     *
     * @param omegaRadPerSec Current robot angular velocity (rad/s); use 0 if unknown
     * @return Rotation rate in rad/s to drive tx toward the center-aim setpoint
     */
    public double calculateAimCorrection(double omegaRadPerSec) {
        if (!hasAnyAllianceTarget()) {
            aimPid.reset();
            return 0.0;
        }
        double tx = Limelight.tx();
        double ty = Limelight.ty();
        double totalLatencyMs = Limelight.tl() + Limelight.cl();
        double omegaDegS = Math.toDegrees(omegaRadPerSec);
        double predictedTx = tx - omegaDegS * totalLatencyMs / 1000.0;

        double distanceM = estimateDistanceToTarget(ty);
        double setpoint = getTxSetpointForCenterAim(distanceM);
        return aimPid.calculate(predictedTx, setpoint);
    }

    /**
     * Calculates the aim correction without latency compensation (omega assumed 0).
     */
    public double calculateAimCorrection() {
        return calculateAimCorrection(0.0);
    }
}
