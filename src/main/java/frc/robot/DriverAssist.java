package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Handles driver assist features including Limelight-based aim assist
 * and shot readiness detection.
 */
public class DriverAssist {
    private final PIDController aimPid;
    private final PIDController distancePid;

    public DriverAssist() {
        // Initialize PID controllers
        aimPid = new PIDController(Constants.AIM_KP, Constants.AIM_KI, Constants.AIM_KD);
        aimPid.setTolerance(Constants.AIM_TOLERANCE);
        aimPid.enableContinuousInput(-180, 180);

        distancePid = new PIDController(Constants.DISTANCE_KP, Constants.DISTANCE_KI, Constants.DISTANCE_KD);
        distancePid.setTolerance(Constants.DISTANCE_TOLERANCE);
    }

    /**
     * Gets the AprilTag IDs for the current alliance.
     */
    public int[] getAllianceTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return new int[0];
        }
        return alliance.get() == DriverStation.Alliance.Blue
            ? Constants.BLUE_HUB_TAG_IDS
            : Constants.RED_HUB_TAG_IDS;
    }

    /**
     * Checks if any alliance-specific AprilTag is visible.
     */
    public boolean hasAnyAllianceTarget() {
        int[] targetIds = getAllianceTagIds();
        for (int tagId : targetIds) {
            if (Limelight.hasTarget(tagId)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if the robot is ready to shoot.
     * Requirements: alliance AprilTag visible, aligned horizontally, at correct distance.
     */
    public boolean isShotReady() {
        if (!hasAnyAllianceTarget()) {
            return false;
        }

        double tx = Limelight.tx();
        double ta = Limelight.ta();

        boolean aligned = Math.abs(tx) <= Constants.READY_TX_DEG;
        boolean atDistance = Math.abs(ta - Constants.DESIRED_TA) <= Constants.READY_TA_TOL;

        return aligned && atDistance;
    }

    /**
     * Calculates shooter voltage based on Limelight distance to AprilTag.
     *
     * Uses linear interpolation between calibrated distance points:
     * - ta >= 4.0 (very close):  6.0V
     * - ta = 2.5 (optimal):      8.0V
     * - ta = 1.5 (far):         10.0V
     * - ta = 0.75 (very far):   11.5V
     * - ta < 0.75 (max range):  12.0V
     */
    public double getShooterVoltageFromLimelight() {
        if (!hasAnyAllianceTarget()) {
            return 0.0;
        }

        double ta = Limelight.ta();

        if (ta >= Constants.TA_VERY_CLOSE) {
            return Constants.VOLTAGE_VERY_CLOSE;
        } else if (ta >= Constants.TA_OPTIMAL) {
            return Constants.VOLTAGE_VERY_CLOSE + (Constants.TA_VERY_CLOSE - ta) * Constants.SLOPE_VERY_CLOSE_TO_OPTIMAL;
        } else if (ta >= Constants.TA_FAR) {
            return Constants.VOLTAGE_OPTIMAL + (Constants.TA_OPTIMAL - ta) * Constants.SLOPE_OPTIMAL_TO_FAR;
        } else if (ta >= Constants.TA_VERY_FAR) {
            return Constants.VOLTAGE_FAR + (Constants.TA_FAR - ta) * Constants.SLOPE_FAR_TO_VERY_FAR;
        } else {
            return Constants.VOLTAGE_MAX;
        }
    }

    /**
     * Resets the aim PID controller.
     */
    public void resetAimPid() {
        aimPid.reset();
    }

    /**
     * Calculates the aim correction based on Limelight data.
     * @return PID-corrected rotation rate
     */
    public double calculateAimCorrection() {
        double tx = Limelight.tx();
        return aimPid.calculate(tx, 0.0);
    }

    /**
     * Gets the aim PID controller (for advanced use).
     */
    public PIDController getAimPid() {
        return aimPid;
    }

    /**
     * Gets the distance PID controller (for advanced use).
     */
    public PIDController getDistancePid() {
        return distancePid;
    }
}
