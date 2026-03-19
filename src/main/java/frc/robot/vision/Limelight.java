package frc.robot.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
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
    /**
     * rawfiducials: flat array of 7 doubles per detected tag:
     * [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity, ...]
     * Works in both single-tag and MegaTag2 modes — unlike tid which is -1 in MegaTag2.
     */
    private static final DoubleArraySubscriber rawFiducialsSub =
            TABLE.getDoubleArrayTopic("rawfiducials").subscribe(new double[0]);

    /**
     * Returns true if the Limelight is actively publishing NT data.
     * Uses the tv subscriber timestamp: timestamp==0 means only the default value
     * has ever been seen (i.e. the limelight has never written to NT).
     */
    public static boolean isConnected() {
        return tvSub.getAtomic().timestamp > 0;
    }

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

    /** Raw fiducials array from NT (7 doubles per tag). Empty if no tags visible or not connected. */
    public static double[] getRawFiducials() {
        return rawFiducialsSub.get();
    }

    /**
     * Returns [txnc, tync] for the hub tag with the largest ta (closest/biggest) among tagIds.
     * Uses rawfiducials so it works in MegaTag2 mode.
     * Returns null if no matching tag is visible.
     * rawfiducials layout per tag: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
     */
    public static double[] getBestTagTxTy(int[] tagIds) {
        double[] raw = rawFiducialsSub.get();
        double bestTa = -1;
        double[] best = null;
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            for (int id : tagIds) {
                if (id == seenId) {
                    double ta = raw[i + 3];
                    if (ta > bestTa) {
                        bestTa = ta;
                        best = new double[]{ raw[i + 1], raw[i + 2] }; // txnc, tync
                    }
                    break;
                }
            }
        }
        return best;
    }

    /**
     * Returns true if any tag in the given ID list is currently visible,
     * using rawfiducials. Works correctly in MegaTag2 mode where tid==-1.
     */
    public static boolean hasAnyTag(int[] tagIds) {
        double[] raw = rawFiducialsSub.get();
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            for (int id : tagIds) {
                if (id == seenId) return true;
            }
        }
        return false;
    }

    /**
     * Returns the tag ID (from tagIds) with the largest ta, or -1 if none visible.
     * Used to lock onto a specific tag for stable alignment across rotation.
     */
    public static int getBestTagId(int[] tagIds) {
        double[] raw = rawFiducialsSub.get();
        double bestTa = -1;
        int bestId = -1;
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            for (int id : tagIds) {
                if (id == seenId) {
                    double ta = raw[i + 3];
                    if (ta > bestTa) {
                        bestTa = ta;
                        bestId = seenId;
                    }
                    break;
                }
            }
        }
        return bestId;
    }

    /**
     * Returns the IDs of the two best visible tags from tagIds (by ta).
     * result[1] == -1 if fewer than two matching tags are visible.
     * Returns [-1, -1] if none are visible.
     */
    public static int[] getBestTwoTagIds(int[] tagIds) {
        double[] raw = rawFiducialsSub.get();
        double best1Ta = -1, best2Ta = -1;
        int best1Id = -1, best2Id = -1;
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            for (int id : tagIds) {
                if (id == seenId) {
                    double ta = raw[i + 3];
                    if (ta > best1Ta) {
                        best2Ta = best1Ta; best2Id = best1Id;
                        best1Ta = ta;     best1Id = seenId;
                    } else if (ta > best2Ta) {
                        best2Ta = ta; best2Id = seenId;
                    }
                    break;
                }
            }
        }
        return new int[]{ best1Id, best2Id };
    }

    /**
     * Returns [txnc, tync] averaged over whichever of id1/id2 are currently visible.
     * If both are visible, returns their midpoint (center between the two tags).
     * If only one is visible, returns that tag's data.
     * id2 may be -1 to track only a single tag.
     * Returns null if neither tag is visible.
     */
    public static double[] getCenterTxTyForIds(int id1, int id2) {
        double[] raw = rawFiducialsSub.get();
        double tx1 = Double.NaN, ty1 = Double.NaN;
        double tx2 = Double.NaN, ty2 = Double.NaN;
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            if (seenId == id1) {
                tx1 = raw[i + 1]; ty1 = raw[i + 2];
            } else if (id2 != -1 && seenId == id2) {
                tx2 = raw[i + 1]; ty2 = raw[i + 2];
            }
        }
        boolean has1 = !Double.isNaN(tx1);
        boolean has2 = !Double.isNaN(tx2);
        if (!has1 && !has2) return null;
        if (has1 && has2) return new double[]{ (tx1 + tx2) / 2.0, (ty1 + ty2) / 2.0 };
        return has1 ? new double[]{ tx1, ty1 } : new double[]{ tx2, ty2 };
    }

    /**
     * Returns [txnc, tync] for whichever of id1/id2 has the smallest distToRobot.
     * Falls back to the single visible tag if only one is visible.
     * id2 may be -1 to track only a single tag.
     * Returns null if neither tag is visible.
     */
    public static double[] getClosestTagTxTyForIds(int id1, int id2) {
        double[] raw = rawFiducialsSub.get();
        double dist1 = Double.MAX_VALUE, dist2 = Double.MAX_VALUE;
        double tx1 = Double.NaN, ty1 = Double.NaN;
        double tx2 = Double.NaN, ty2 = Double.NaN;
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            if (seenId == id1) {
                tx1 = raw[i + 1]; ty1 = raw[i + 2]; dist1 = raw[i + 5];
            } else if (id2 != -1 && seenId == id2) {
                tx2 = raw[i + 1]; ty2 = raw[i + 2]; dist2 = raw[i + 5];
            }
        }
        boolean has1 = !Double.isNaN(tx1);
        boolean has2 = !Double.isNaN(tx2);
        if (!has1 && !has2) return null;
        if (has1 && has2) return dist1 <= dist2 ? new double[]{ tx1, ty1 } : new double[]{ tx2, ty2 };
        return has1 ? new double[]{ tx1, ty1 } : new double[]{ tx2, ty2 };
    }

    /**
     * Returns [txnc, tync] for the given tag ID, or null if not currently visible.
     * rawfiducials layout per tag: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
     */
    public static double[] getTagTxTy(int tagId) {
        double[] raw = rawFiducialsSub.get();
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            if ((int) raw[i] == tagId) {
                return new double[]{ raw[i + 1], raw[i + 2] };
            }
        }
        return null;
    }

    /**
     * Selects the best hub tag to lock onto, handling ambiguous pairs.
     *
     * Algorithm:
     *   1. Collect all visible tags from hubTagIds.
     *   2. For each ambiguous pair where BOTH tags are visible, exclude the one with the
     *      larger |txnc| (farther from camera center). Ties go to the first of the pair.
     *   3. Among the remaining candidates, return the one with the largest ta (closest tag).
     *
     * Returns -1 if no hub tag is visible.
     * rawfiducials layout per tag: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
     */
    public static int getBestHubTagId(int[] hubTagIds, int[][] ambiguousPairs) {
        double[] raw = rawFiducialsSub.get();
        int maxVisible = hubTagIds.length;

        // Parallel arrays for visible hub tags — avoids per-cycle heap allocation
        int[]     seenIds  = new int[maxVisible];
        double[]  seenTx   = new double[maxVisible];
        double[]  seenTa   = new double[maxVisible];
        boolean[] excluded = new boolean[maxVisible];
        int count = 0;

        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int id = (int) raw[i];
            for (int hubId : hubTagIds) {
                if (id == hubId) {
                    seenIds[count]  = id;
                    seenTx[count]   = raw[i + 1]; // txnc
                    seenTa[count]   = raw[i + 3]; // ta
                    count++;
                    break;
                }
            }
        }

        if (count == 0) return -1;

        // Resolve ambiguous pairs: exclude the tag that is farther off-center
        for (int[] pair : ambiguousPairs) {
            int idx0 = -1, idx1 = -1;
            for (int j = 0; j < count; j++) {
                if      (seenIds[j] == pair[0]) idx0 = j;
                else if (seenIds[j] == pair[1]) idx1 = j;
            }
            if (idx0 != -1 && idx1 != -1) {
                // Both visible — keep the one with smaller |tx| (more centered)
                if (Math.abs(seenTx[idx0]) <= Math.abs(seenTx[idx1])) {
                    excluded[idx1] = true;
                } else {
                    excluded[idx0] = true;
                }
            }
        }

        // Pick largest ta (closest) among non-excluded candidates
        double bestTa = -1;
        int bestId = -1;
        for (int j = 0; j < count; j++) {
            if (!excluded[j] && seenTa[j] > bestTa) {
                bestTa = seenTa[j];
                bestId = seenIds[j];
            }
        }
        return bestId;
    }

    /**
     * Returns the tag ID (from tagIds) with the smallest ta (farthest visible), or -1 if none.
     * Use this to lock onto a specific hub tag at alignment start.
     * rawfiducials layout per tag: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
     */
    public static int getSmallestTaTagId(int[] tagIds) {
        double[] raw = rawFiducialsSub.get();
        double smallestTa = Double.MAX_VALUE;
        int result = -1;
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            int seenId = (int) raw[i];
            for (int id : tagIds) {
                if (id == seenId) {
                    double ta = raw[i + 3];
                    if (ta < smallestTa) { smallestTa = ta; result = seenId; }
                    break;
                }
            }
        }
        return result;
    }

    /**
     * Checks if the Limelight has a valid target with the specified AprilTag ID.
     * Uses rawfiducials so it works in MegaTag2 mode (tid is unreliable there).
     */
    public static boolean hasTarget(int targetTagId) {
        double[] raw = rawFiducialsSub.get();
        for (int i = 0; i + 7 <= raw.length; i += 7) {
            if ((int) raw[i] == targetTagId) return true;
        }
        return false;
    }
}
