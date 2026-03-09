package frc.robot;

import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

/**
 * Factory methods for autonomous routines.
 *
 * Auto 1 — Hub Score:
 *   Drive toward the hub with Limelight aim correction, then hold aim and shoot for 10 s.
 *
 * Auto 2 — Hub Score + Return:
 *   Same as Auto 1, then rotate 360° and pathfind back to the human player station.
 *   The target pose is derived from the human player station AprilTag IDs via the
 *   WPILib field layout — no hardcoded field coordinates.
 */
public final class Autos {
    private Autos() {}

    /** Forward speed (m/s) while approaching the hub. */
    private static final double DRIVE_TO_HUB_SPEED_MPS = 1.0;

    /** Limelight TA at which the robot stops approaching and starts shooting. */
    private static final double HUB_TA_THRESHOLD = 0.2;

    /** Safety timeout for the approach phase in case the hub is never acquired. */
    private static final double APPROACH_TIMEOUT_S = 5.0;

    /** How long to hold aim and shoot at the hub. */
    private static final double SHOOT_DURATION_S = 10.0;

    /** How far in front of the station AprilTag to stop (meters). */
    private static final double STATION_APPROACH_DISTANCE_M = 1.5;

    /** PathPlanner speed/accel limits for the return-to-station path. */
    private static final PathConstraints RETURN_CONSTRAINTS =
            new PathConstraints(2.0, 2.0, Math.PI, 2.0 * Math.PI);

    // -------------------------------------------------------------------------
    // Auto 1: Hub Score
    // -------------------------------------------------------------------------

    /**
     * Drives toward the hub with Limelight aim correction until TA ≥ {@value #HUB_TA_THRESHOLD},
     * then holds aim, spins shooter to Limelight-derived RPS, and feeds for {@value #SHOOT_DURATION_S} s.
     */
    public static Command scoreAtHub(
            CommandSwerveDrivetrain drivetrain, Shooter shooter, DriverAssist driverAssist) {

        // Robot-centric for approach: +X = forward (toward wherever the robot faces).
        // The aim correction keeps the robot facing the hub, so this works for both alliances
        // without needing to know which direction the hub is in field coordinates.
        SwerveRequest.RobotCentric approachDrive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(0)
                .withRotationalDeadband(0);

        SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(0)
                .withRotationalDeadband(0);

        // Phase 1: drive forward + aim correction until TA reaches the scoring zone (or 5 s timeout)
        Command approach = drivetrain.applyRequest(() -> {
            double omega = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
            double turn  = MathUtil.clamp(
                    driverAssist.calculateAimCorrection(omega),
                    -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
            return approachDrive
                    .withVelocityX(DRIVE_TO_HUB_SPEED_MPS)
                    .withVelocityY(0)
                    .withRotationalRate(-turn);
        })
        .until(() -> driverAssist.hasAnyAllianceTarget() && Limelight.ta() >= HUB_TA_THRESHOLD)
        .withTimeout(APPROACH_TIMEOUT_S);

        // Phase 2: hold aim (rotation only) + shoot + feed for 10 s
        Command alignAndShoot = Commands.parallel(
            drivetrain.applyRequest(() -> {
                double omega = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
                double turn  = MathUtil.clamp(
                        driverAssist.calculateAimCorrection(omega),
                        -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
                return aimDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(-turn);
            }),
            Commands.runEnd(
                () -> {
                    double rps = driverAssist.getShooterTargetRPSFromLimelight();
                    shooter.runShoot(rps > 0 ? rps : Constants.Shooter.SHOOTER_DEFAULT_RPS);
                    shooter.runFeed();
                },
                shooter::stop,
                shooter
            )
        ).withTimeout(SHOOT_DURATION_S)
         .onlyIf(driverAssist::hasAnyAllianceTarget); // skip blind shooting if approach never found the hub

        return Commands.sequence(
            Commands.runOnce(() -> {
                Limelight.setPipeline(Constants.LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                driverAssist.resetAimPid();
            }),
            approach,
            alignAndShoot
        );
    }

    // -------------------------------------------------------------------------
    // Auto 2: Hub Score + Return
    // -------------------------------------------------------------------------

    /**
     * Runs {@link #scoreAtHub}, rotates 360°, then pathfinds to the human player station.
     *
     * <p>The target pose is computed from the human player station AprilTag IDs defined in
     * {@link Constants}: the WPILib field layout provides the exact field position of the tag,
     * and the robot navigates to a point {@value #STATION_APPROACH_DISTANCE_M} m in front of it.
     * Pose estimation during navigation uses MegaTag (AprilTag-based).
     */
    public static Command scoreAndReturn(
            CommandSwerveDrivetrain drivetrain, Shooter shooter, DriverAssist driverAssist) {

        SwerveRequest.FieldCentric rotateDrive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(0)
                .withRotationalDeadband(0);

        // Rotate 360° — use 1.5× theoretical time to account for open-loop acceleration lag
        double rotateTime = (2 * Math.PI) / Constants.MAX_ANGULAR_RATE * 1.5;
        Command rotate360 = drivetrain.applyRequest(
                () -> rotateDrive.withVelocityX(0).withVelocityY(0)
                                 .withRotationalRate(Constants.MAX_ANGULAR_RATE)
        ).withTimeout(rotateTime);

        // Deferred: alliance and field layout are resolved at scheduling time, not construction time
        Command returnToStation = Commands.defer(
            () -> {
                int[] tagIds = DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                        ? Constants.BLUE_HUMAN_PLAYER_TAG_IDS
                        : Constants.RED_HUMAN_PLAYER_TAG_IDS;

                Optional<Pose2d> target = stationPoseFromLayout(tagIds);
                if (target.isEmpty()) {
                    return Commands.none();
                }
                return AutoBuilder.pathfindToPose(target.get(), RETURN_CONSTRAINTS, 0.0);
            },
            Set.of((Subsystem) drivetrain)
        );

        return Commands.sequence(
            scoreAtHub(drivetrain, shooter, driverAssist),
            rotate360,
            returnToStation
        );
    }

    // -------------------------------------------------------------------------
    // Helper
    // -------------------------------------------------------------------------

    /**
     * Looks up the first matching tag ID in the WPILib field layout and returns a target
     * pose {@value #STATION_APPROACH_DISTANCE_M} m in front of it, robot facing the tag.
     * Returns empty if the field layout cannot be loaded or no tag is found.
     */
    private static Optional<Pose2d> stationPoseFromLayout(int[] tagIds) {
        AprilTagFieldLayout layout;
        try {
            // k2026RebuiltAndymark is the standard competition field layout
            layout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
        } catch (Exception e) {
            return Optional.empty();
        }

        for (int id : tagIds) {
            var tagPose3d = layout.getTagPose(id);
            if (tagPose3d.isPresent()) {
                Pose2d tag = tagPose3d.get().toPose2d();

                // Offset in front of the tag (along the tag's facing direction)
                Translation2d offset = new Translation2d(STATION_APPROACH_DISTANCE_M, 0)
                        .rotateBy(tag.getRotation());

                // Robot faces back toward the tag
                Rotation2d robotHeading = tag.getRotation().rotateBy(Rotation2d.fromDegrees(180));

                return Optional.of(new Pose2d(tag.getTranslation().plus(offset), robotHeading));
            }
        }
        return Optional.empty();
    }
}
