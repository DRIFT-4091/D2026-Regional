package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

/**
 * Operator Interface - handles all controller bindings for GameSir G7 SE (Xbox layout).
 * Left stick: strafe + forward/back. Right stick: rotation.
 * LT=intake, RT=shooter, LB=reset field, RB=driver assist, R4=inverse shooter, L4=inverse intake, X=resistance.
 */
public class OI {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final DriverAssist driverAssist;
    private final Telemetry logger;

    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.SwerveDriveBrake brake;
    private final SwerveRequest.PointWheelsAt point;

    public OI(CommandSwerveDrivetrain drivetrain, DriverAssist driverAssist, Telemetry logger, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.driverAssist = driverAssist;
        this.logger = logger;
        this.joystick = new CommandXboxController(Constants.CONTROLLER_PORT);

        this.drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DRIVE_DEADBAND)
                .withRotationalDeadband(Constants.ROTATION_DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.brake = new SwerveRequest.SwerveDriveBrake();
        this.point = new SwerveRequest.PointWheelsAt();

        configureBindings();
    }

    private void configureBindings() {
        configureDefaultCommands();
        configureDrivetrainControls();
        configureShooterControls();
        configureDriverAssist();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.MAX_SPEED)
                    .withVelocityY(-joystick.getLeftX() * Constants.MAX_SPEED)
                    .withRotationalRate(-joystick.getRightX() * Constants.MAX_ANGULAR_RATE)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        RobotModeTriggers.disabled().onFalse(
            Commands.runOnce(drivetrain::seedFieldCentric).andThen(
                drivetrain.applyRequest(() ->
                    point.withModuleDirection(new Rotation2d(0))
                ).withTimeout(Constants.FIELD_CENTRIC_SEED_TIMEOUT_S)
            )
        );
    }

    private void configureDrivetrainControls() {
        // LB = reset field-centric heading
        triggerButton(Constants.LEFT_BUTTON).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // X = resistance mode (brake wheels while held)
        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
    }

    private void configureShooterControls() {
        // LT = intake
        joystick.leftTrigger().whileTrue(
            Commands.runEnd(shooter::runIntake, shooter::stop, shooter)
        );

        // Shooter + feeder in one command so they don't interrupt each other. RT = wheel at RPS, Y = feeder (when target visible).
        // Rumble when at target RPS while holding RT (not Y); turn off rumble when Y is pressed or when command ends.
        joystick.rightTrigger().or(joystick.y()).whileTrue(
            Commands.runEnd(
                () -> {
                    double rps = driverAssist.getShooterTargetRPSFromLimelight();
                    boolean rt = joystick.rightTrigger().getAsBoolean();
                    boolean y = joystick.y().getAsBoolean();

                    if (rt) {
                        shooter.runShoot(rps);
                    } else {
                        shooter.stop();
                    }
                    if (y && rps > 0) {
                        shooter.runFeed();
                        joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    } else {
                        shooter.stopFeeder();
                        // Rumble when RT only (no Y), target > 0, and shooter at target RPS
                        if (rt && rps > 0) {
                            double actual = shooter.getShooterVelocity();
                            boolean atTarget = Math.abs(actual - rps) <= Constants.Shooter.SHOOTER_RPS_RUMBLE_TOLERANCE;
                            joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble,
                                    atTarget ? Constants.Shooter.SHOOTER_RUMBLE_STRENGTH : 0);
                        } else {
                            joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                        }
                    }
                },
                () -> {
                    shooter.stop();
                    joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                },
                shooter
            )
        );

        joystick.b().whileTrue(
            Commands.runEnd(shooter::runFeedReverse, shooter::stop, shooter)
        );

        joystick.a().whileTrue(
            Commands.runEnd(shooter::runSystemReverse, shooter::stop, shooter)
        );
    }

    private void configureDriverAssist() {
        // RB = driver assist (AprilTag aim)
        triggerButton(Constants.RIGHT_BUTTON).whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(Constants.LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                driverAssist.resetAimPid();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vx = -joystick.getLeftY() * Constants.MAX_SPEED;
                    double vy = -joystick.getLeftX() * Constants.MAX_SPEED;
                    double omega = -joystick.getRightX() * Constants.MAX_ANGULAR_RATE;

                    if (driverAssist.hasAnyAllianceTarget()) {
                        double omegaRadS = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
                        double turnCmd = driverAssist.calculateAimCorrection(omegaRadS);
                        turnCmd = MathUtil.clamp(turnCmd, -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
                        omega = turnCmd;
                    }

                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(omega);
                })
            )
        );
    }

    /** Trigger for a raw button (e.g. GameSir back paddles, LB/RB if mapped as raw). */
    private Trigger triggerButton(int buttonId) {
        return new Trigger(() -> joystick.getHID().getRawButton(buttonId));
    }
}
