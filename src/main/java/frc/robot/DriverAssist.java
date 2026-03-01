package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Handles driver assist features including Limelight-based aim assist.
 */
public class DriverAssist {
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
            int[] both = new int[Constants.BLUE_HUB_TAG_IDS.length + Constants.RED_HUB_TAG_IDS.length];
            System.arraycopy(Constants.BLUE_HUB_TAG_IDS, 0, both, 0, Constants.BLUE_HUB_TAG_IDS.length);
            System.arraycopy(Constants.RED_HUB_TAG_IDS, 0, both, Constants.BLUE_HUB_TAG_IDS.length, Constants.RED_HUB_TAG_IDS.length);
            return both;
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
     * Target shooter velocity (rotations per second) from Limelight TA.
     * Used for velocity-controlled shooting so RPM is held under load.
     * Returns 0 if no AprilTag (shooter should stop).
     */
    public double getShooterTargetRPSFromLimelight() {
        if (!hasAnyAllianceTarget()) {
            return 0.0;
        }

        double ta = Limelight.ta();

        if (ta >= Constants.Shooter.TA_BAND_CLOSE_LO) {
            return Constants.Shooter.SHOOTER_RPS_TA_06_08;
        } else if (ta >= Constants.Shooter.TA_BAND_MID_HI_LO) {
            return Constants.Shooter.SHOOTER_RPS_TA_04_06;
        } else if (ta >= Constants.Shooter.TA_BAND_FAR_LO) {
            return Constants.Shooter.SHOOTER_RPS_TA_02_04;
        } else {
            return Constants.Shooter.SHOOTER_RPS_TA_00_02;
        }
    }

    /**
     * Resets the aim PID controller.
     */
    public void resetAimPid() {
        aimPid.reset();
    }

    /**
     * Calculates the aim correction based on Limelight data (tx = horizontal offset to target).
     * @return Rotation rate in rad/s to drive tx toward 0
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
}
