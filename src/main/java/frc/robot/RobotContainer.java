package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.25)
            .withRotationalDeadband(MaxAngularRate * 0.25)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // PS4 controller (change to 0 if needed)
    private final CommandPS4Controller joystick = new CommandPS4Controller(1);

    private final Shooter shooter = new Shooter();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // ---------------- LIMELIGHT / DRIVER ASSIST SETTINGS ----------------
    private static final int LL_AIM_PIPELINE = 0;

    // REBUILT FRC Challenge AprilTag IDs
    private static final int[] BLUE_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
    private static final int[] RED_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

    // PID Controllers
    private final PIDController aimPid = new PIDController(0.06, 0.0, 0.001);
    private final PIDController distancePid = new PIDController(0.6, 0.0, 0.0);

    // Alignment thresholds
    private static final double DESIRED_TA = 2.0;           // Target area at optimal shooting distance
    private static final double READY_TX_DEG = 1.0;         // Horizontal alignment tolerance (degrees)
    private static final double READY_TA_TOL = 0.15;        // Distance tolerance (target area)
    private static final double MAX_AIM_RAD_PER_SEC = 2.5;  // Max rotation speed for aim assist
    private static final double READY_RUMBLE = 1.0;         // Controller rumble intensity when ready


    public RobotContainer() {
        aimPid.setTolerance(1.0);
        aimPid.enableContinuousInput(-180, 180);

        distancePid.setTolerance(0.15);

        configureBindings();
    }

    private void configureBindings() {

        // ---------------- DEFAULT DRIVE (MANUAL) ----------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // ---------------- DRIVETRAIN CONTROLS ----------------
        // Cross = brake wheels
        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // Circle = point wheels (manual wheel direction)
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // L1 = reset field-centric heading
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ---------------- SHOOTER CONTROLS ----------------
        // R2 = shoot (auto voltage from Limelight distance)
        joystick.R2().whileTrue(
            Commands.runEnd(() -> shooter.runShoot(getShooterVoltageFromLimelight()), shooter::stop, shooter)
        );

        // L2 = intake
        joystick.L2().whileTrue(
            Commands.runEnd(shooter::runIntake, shooter::stop, shooter)
        );

        // L1 + Circle = unjam feeder (reverse)
        joystick.L1().and(joystick.circle()).whileTrue(
            Commands.runEnd(shooter::runFeedReverse, shooter::stop, shooter)
        );

        // L1 + Square = unjam intake (reverse)
        joystick.L1().and(joystick.square()).whileTrue(
            Commands.runEnd(shooter::runIntakeReverse, shooter::stop, shooter)
        );

        // ---------------- SYSTEM IDENTIFICATION (SysId) ----------------
        // Share + Triangle = dynamic forward, Share + Square = dynamic reverse
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // Options + Triangle = quasistatic forward, Options + Square = quasistatic reverse
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ---------------- LIMELIGHT: AIM ASSIST (R1) ----------------
        joystick.R1().whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                aimPid.reset();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vx = joystick.getLeftY() * MaxSpeed;
                    double vy = -joystick.getLeftX() * MaxSpeed;

                    double omega = -joystick.getRightX() * MaxAngularRate;

                    // Auto-aim if any alliance AprilTag is visible
                    if (hasAnyAllianceTarget()) {
                        double tx = Limelight.tx();
                        double turnCmd = aimPid.calculate(tx, 0.0);
                        turnCmd = MathUtil.clamp(turnCmd, -MAX_AIM_RAD_PER_SEC, MAX_AIM_RAD_PER_SEC);
                        omega = turnCmd;
                    }

                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(omega);
                })
            )
        );

        // ---------------- READY-TO-SHOOT RUMBLE (GLOBAL) ----------------
        Trigger shotReady = new Trigger(() ->
            DriverStation.isTeleopEnabled() && isShotReady()
        );

        shotReady.whileTrue(
            Commands.run(() ->
                joystick.setRumble(RumbleType.kBothRumble, READY_RUMBLE)
            )
        );

        shotReady.onFalse(
            Commands.runOnce(() ->
                joystick.setRumble(RumbleType.kBothRumble, 0.0)
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Gets the AprilTag IDs for the current alliance.
     *
     * @return Array of AprilTag IDs for current alliance, or empty array if no alliance
     */
    private int[] getAllianceTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return new int[0];
        }
        return alliance.get() == DriverStation.Alliance.Blue
            ? BLUE_HUB_TAG_IDS
            : RED_HUB_TAG_IDS;
    }

    /**
     * Checks if any alliance-specific AprilTag is visible.
     *
     * @return true if any alliance AprilTag is detected
     */
    private boolean hasAnyAllianceTarget() {
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
     * Requirements:
     * 1. Alliance AprilTag is visible
     * 2. Robot is aligned horizontally (tx within tolerance)
     * 3. Robot is at correct distance (ta within tolerance)
     *
     * Note: Does NOT check shooter motor speed - motor only spins when R2 is pressed.
     *
     * @return true if robot is positioned correctly to shoot
     */
    private boolean isShotReady() {
        // Check vision target
        if (!hasAnyAllianceTarget()) {
            return false;
        }

        // Check alignment and distance
        double tx = Limelight.tx();
        double ta = Limelight.ta();

        boolean aligned = Math.abs(tx) <= READY_TX_DEG;
        boolean atDistance = Math.abs(ta - DESIRED_TA) <= READY_TA_TOL;

        return aligned && atDistance;
    }

    /**
     * Calculates shooter voltage based on Limelight distance to AprilTag.
     * Uses REBUILT FRC Challenge AprilTag IDs for blue and red alliances.
     *
     * Distance-to-voltage mapping (tunable based on testing):
     * - ta >= 4.0 (very close): 6.0V
     * - ta = 2.0 (optimal):     9.0V
     * - ta = 1.0 (far):         11.0V
     * - ta <= 0.75 (very far):  12.0V
     *
     * @return Shooter voltage (0.0 if no valid target detected)
     */
    public double getShooterVoltageFromLimelight() {
        // Check if any alliance AprilTag is visible
        if (!hasAnyAllianceTarget()) {
            return 0.0;
        }

        // Get distance measurement from Limelight (ta = target area %)
        double ta = Limelight.ta();

        // Calculate voltage based on distance (closer = lower voltage, farther = higher voltage)
        if (ta >= 4.0) {
            return 6.0;
        } else if (ta >= 2.5) {
            return 6.0 + (4.0 - ta) * (2.0 / 1.5);  // Interpolate 6.0V to 8.0V
        } else if (ta >= 1.5) {
            return 8.0 + (2.5 - ta) * (2.0 / 1.0);  // Interpolate 8.0V to 10.0V
        } else if (ta >= 0.75) {
            return 10.0 + (1.5 - ta) * (1.5 / 0.75); // Interpolate 10.0V to 11.5V
        } else {
            return 12.0; // Maximum voltage for very far shots
        }
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}



