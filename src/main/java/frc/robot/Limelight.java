

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Limelight {
    private Limelight() {}

    // Change this if your Limelight name is not "limelight"
    private static final String TABLE_NAME = "limelight";
    private static NetworkTable table() {
        return NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    }

    /** Whether the Limelight has a valid target (tv == 1). */
    public static boolean hasTarget() {
        return table().getEntry("tv").getDouble(0) == 1.0;
    }

    /** Horizontal offset from crosshair to target (degrees). */
    public static double tx() {
        return table().getEntry("tx").getDouble(0.0);
    }

    /** Vertical offset from crosshair to target (degrees). */
    public static double ty() {
        return table().getEntry("ty").getDouble(0.0);
    }

    /** Target area (0% to 100% of image). */
    public static double ta() {
        return table().getEntry("ta").getDouble(0.0);
    }

    /** Set pipeline index. */
    public static void setPipeline(int pipeline) {
        table().getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * LED mode: 0=use pipeline, 1=force off, 2=blink, 3=force on
     */
    public static void setLedMode(int mode) {
        table().getEntry("ledMode").setNumber(mode);
    }

    /**
     * Checks if the Limelight has a valid target with the specified AprilTag ID.
     *
     * @param targetTagId The AprilTag ID to check for
     * @return true if a valid target is detected and matches the specified tag ID
     */
    public static boolean hasTarget(int targetTagId) {
        if (!hasTarget()) {
            return false; // No target detected at all
        }

        // Get the detected AprilTag ID from NetworkTables
        // The 'tid' field contains the ID of the primary detected AprilTag
        double detectedTagId = table().getEntry("tid").getDouble(-1);

        return detectedTagId == targetTagId;
    }

}
