package frc.robot.vision;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;

public final class Limelight {
    private Limelight() {}

    private static final NetworkTable TABLE =
            NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_TABLE_NAME);

    /* NT4 typed subscribers (cached, no string lookup per read) */
    private static final DoubleSubscriber tvSub = TABLE.getDoubleTopic("tv").subscribe(0.0);
    private static final DoubleSubscriber txSub = TABLE.getDoubleTopic("tx").subscribe(0.0);
    private static final DoubleSubscriber tySub = TABLE.getDoubleTopic("ty").subscribe(0.0);
    private static final DoubleSubscriber taSub = TABLE.getDoubleTopic("ta").subscribe(0.0);
    private static final DoubleSubscriber tidSub = TABLE.getDoubleTopic("tid").subscribe(-1.0);
    private static final DoubleSubscriber tlSub = TABLE.getDoubleTopic("tl").subscribe(0.0);
    private static final DoubleSubscriber clSub = TABLE.getDoubleTopic("cl").subscribe(0.0);

    /** Whether the Limelight has a valid target (tv == 1). */
    public static boolean hasTarget() {
        return tvSub.get() == 1.0;
    }

    /** Horizontal offset from crosshair to target (degrees). */
    public static double tx() {
        return txSub.get();
    }

    /** Vertical offset from crosshair to target (degrees). */
    public static double ty() {
        return tySub.get();
    }

    /** Target area (0% to 100% of image). */
    public static double ta() {
        return taSub.get();
    }

    /** AprilTag ID of primary detected target (dimensionless). -1 if no target. */
    public static double getTid() {
        return tidSub.get();
    }

    /** Pipeline latency (ms). */
    public static double tl() {
        return tlSub.get();
    }

    /** Capture latency (ms). */
    public static double cl() {
        return clSub.get();
    }

    /** Set pipeline index. */
    public static void setPipeline(int pipeline) {
        TABLE.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * LED mode: 0=use pipeline, 1=force off, 2=blink, 3=force on
     */
    public static void setLedMode(int mode) {
        TABLE.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Checks if the Limelight has a valid target with the specified AprilTag ID.
     * Single NT read (getTid returns -1 when no target).
     */
    public static boolean hasTarget(int targetTagId) {
        return getTid() == targetTagId;
    }
}
