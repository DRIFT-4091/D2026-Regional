package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

/**
 * Generates Limelight .fmap files from WPILib AprilTag field layouts (e.g. 2026 REBUILT).
 * Run main() to write 2026-rebuilt-welded.fmap and 2026-rebuilt-andymark.fmap to src/main/deploy.
 * Upload the .fmap to your Limelight via the web UI or REST API for MegaTag2.
 */
public final class FmapGenerator {

    /** FRC standard AprilTag size in mm (36h11 classic). */
    private static final double TAG_SIZE_MM = 165.1;
    private static final String FAMILY = "apriltag3_36h11_classic";

    private FmapGenerator() {}

    /**
     * Converts a WPILib Pose3d to a 4x4 row-major transform array (SI units) for Limelight .fmap.
     */
    private static double[] pose3dToRowMajor4x4(Pose3d pose) {
        Matrix<N3, N3> r = pose.getRotation().toMatrix();
        double tx = pose.getTranslation().getX();
        double ty = pose.getTranslation().getY();
        double tz = pose.getTranslation().getZ();
        return new double[] {
            r.get(0, 0), r.get(0, 1), r.get(0, 2), tx,
            r.get(1, 0), r.get(1, 1), r.get(1, 2), ty,
            r.get(2, 0), r.get(2, 1), r.get(2, 2), tz,
            0, 0, 0, 1
        };
    }

    private static void appendDoubleArray(StringBuilder sb, double[] arr) {
        sb.append("[");
        for (int i = 0; i < arr.length; i++) {
            if (i > 0) sb.append(",");
            sb.append(arr[i]);
        }
        sb.append("]");
    }

    /**
     * Builds Limelight .fmap JSON from a WPILib AprilTagFieldLayout.
     */
    public static String toFmapJson(AprilTagFieldLayout layout) {
        List<AprilTag> tags = layout.getTags();
        StringBuilder sb = new StringBuilder();
        sb.append("{\"type\":\"frc\",\"fiducials\":[");
        for (int i = 0; i < tags.size(); i++) {
            AprilTag tag = tags.get(i);
            if (i > 0) sb.append(",");
            double[] transform = pose3dToRowMajor4x4(tag.pose);
            sb.append("{\"family\":\"").append(FAMILY).append("\",\"id\":").append(tag.ID)
                .append(",\"size\":").append(TAG_SIZE_MM).append(",\"transform\":");
            appendDoubleArray(sb, transform);
            sb.append(",\"unique\":1}");
        }
        sb.append("]}");
        return sb.toString();
    }

    /**
     * Loads the given field and writes a Limelight .fmap file to the specified path.
     */
    public static void writeFmap(AprilTagFields field, Path outputPath) throws IOException {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(field);
        String json = toFmapJson(layout);
        Files.createDirectories(outputPath.getParent());
        Files.writeString(outputPath, json);
    }

    /**
     * Generates 2026 REBUILT Welded and AndyMark .fmap files into src/main/deploy.
     * Run this once (e.g. from IDE or gradle run with main class set to this) to produce
     * 2026-rebuilt-welded.fmap and 2026-rebuilt-andymark.fmap for upload to the Limelight.
     */
    public static void main(String[] args) {
        Path deployDir = Paths.get("src/main/deploy");
        try {
            writeFmap(AprilTagFields.k2026RebuiltWelded, deployDir.resolve("2026-rebuilt-welded.fmap"));
            System.out.println("Wrote " + deployDir.resolve("2026-rebuilt-welded.fmap"));
            writeFmap(AprilTagFields.k2026RebuiltAndymark, deployDir.resolve("2026-rebuilt-andymark.fmap"));
            System.out.println("Wrote " + deployDir.resolve("2026-rebuilt-andymark.fmap"));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
