package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

/**
 * Operator Interface - single GameSir G7 controller (Xbox layout) on port 0.
 * Left stick: translation. Right stick: rotation.
 * LT: intake. LB: reset heading. RT: shooter. Y: feeder.
 * B: feed reverse. A: system reverse. X: basket wobble. RB: auto-align.
 */
public class OI {
    private final CommandXboxController driverJoystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final DriverAssist driverAssist;
    private final Telemetry logger;

    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.FieldCentric wobbleDrive;
    private final SwerveRequest.FieldCentric aimDrive;

    public OI(CommandSwerveDrivetrain drivetrain, DriverAssist driverAssist, Telemetry logger, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.driverAssist = driverAssist;
        this.logger = logger;
        this.driverJoystick = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

        this.drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DRIVE_DEADBAND)
                .withRotationalDeadband(Constants.ROTATION_DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.wobbleDrive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.aimDrive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DRIVE_DEADBAND)
                .withRotationalDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
            drivetrain.applyRequest(() -> {
                double leftY = MathUtil.applyDeadband(-driverJoystick.getLeftY(), Constants.JOYSTICK_DEADBAND);
                double leftX = MathUtil.applyDeadband(driverJoystick.getLeftX(), Constants.JOYSTICK_DEADBAND);
                double rightX = MathUtil.applyDeadband(-driverJoystick.getHID().getRawAxis(Constants.ROTATION_AXIS), Constants.JOYSTICK_DEADBAND);
                double vx = leftY * Constants.MAX_SPEED;
                double vy = leftX * Constants.MAX_SPEED;
                double omega = rightX * Constants.MAX_ANGULAR_RATE;
                return drive.withVelocityX(vx).withVelocityY(-vy).withRotationalRate(-omega);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Auto-zero heading on teleop enable: reset Pigeon yaw to 0 and sync pose rotation
        RobotModeTriggers.teleop().onTrue(drivetrain.runOnce(() -> {
            drivetrain.getPigeon2().setYaw(0);
            drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(
                drivetrain.getState().Pose.getTranslation(),
                edu.wpi.first.math.geometry.Rotation2d.kZero
            ));
        }));
    }

    private void configureDrivetrainControls() {
        // LB = reset gyro (seed field-centric to current facing direction) - temporary for practice
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // X = basket wobble + rumble
        driverJoystick.x().whileTrue(
            Commands.startEnd(
                () -> driverJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, Constants.Shooter.SHOOTER_RUMBLE_STRENGTH),
                () -> driverJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)
            ).alongWith(drivetrain.applyRequest(() -> {
                double t = Timer.getFPGATimestamp();
                double vx = Constants.WOBBLE_SPEED_MPS * Math.sin(2 * Math.PI * Constants.WOBBLE_HZ * t);
                return wobbleDrive.withVelocityX(vx).withVelocityY(0).withRotationalRate(0);
            }))
        );
    }

    private void configureShooterControls() {
        // LT = intake
        driverJoystick.leftTrigger().whileTrue(
            Commands.runEnd(shooter::runIntake, shooter::stop, shooter)
        );

        // RT = shoot wheel, Y = feeder (only when shooter is at target RPS).
        // Joystick rumbles when shooter is at target RPS and RT is held.
        driverJoystick.rightTrigger().or(driverJoystick.y()).whileTrue(
            Commands.runEnd(
                () -> {
                    double limelightRps = driverAssist.getShooterTargetRPSFromLimelight();
                    double targetRps = limelightRps > 0 ? limelightRps : Constants.Shooter.SHOOTER_DEFAULT_RPS;
                    boolean rt = driverJoystick.rightTrigger().getAsBoolean();
                    boolean y = driverJoystick.y().getAsBoolean();
                    double actual = shooter.getShooterVelocity();
                    boolean atTarget = Math.abs(actual - targetRps) <= Constants.Shooter.SHOOTER_RPS_RUMBLE_TOLERANCE;
                    if (rt) {
                        shooter.runShoot(targetRps);
                    } else {
                        shooter.stop();
                    }
                    if (y && atTarget) {
                        shooter.runFeed();
                        driverJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    } else {
                        shooter.stopFeeder();
                        driverJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble,
                                (rt && atTarget) ? Constants.Shooter.SHOOTER_RUMBLE_STRENGTH : 0);
                    }
                },
                () -> {
                    shooter.stop();
                    driverJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                },
                shooter
            )
        );

        // B = feed reverse
        driverJoystick.b().whileTrue(
            Commands.runEnd(shooter::runFeedReverse, shooter::stop, shooter)
        );

        // A = system reverse
        driverJoystick.a().whileTrue(
            Commands.runEnd(shooter::runSystemReverse, shooter::stop, shooter)
        );
    }

    private void configureDriverAssist() {
        // RB = driver assist (AprilTag aim)
        driverJoystick.rightBumper().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("[OI] Driver assist activated");
                Limelight.setPipeline(Constants.LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                driverAssist.resetAimPid();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double leftY = MathUtil.applyDeadband(driverJoystick.getLeftY(), Constants.JOYSTICK_DEADBAND);
                    double leftX = MathUtil.applyDeadband(driverJoystick.getLeftX(), Constants.JOYSTICK_DEADBAND);
                    double vx = leftY * Constants.MAX_SPEED;
                    double vy = leftX * Constants.MAX_SPEED;
                    double omegaRadS = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
                    double turnCmd = driverAssist.calculateAimCorrection(omegaRadS);
                    turnCmd = MathUtil.clamp(turnCmd, -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);

                    return aimDrive.withVelocityX(vx)
                                  .withVelocityY(-vy)
                                  .withRotationalRate(turnCmd);
                })
            )
        );
    }
}
