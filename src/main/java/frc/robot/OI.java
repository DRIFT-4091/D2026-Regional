package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

/**
 * Operator Interface - two GameSir G7 controllers (Xbox layout).
 * Driver   (port 0): movement, reset heading, auto-align, basket wobble. Rumbles during wobble.
 * Operator (port 1): intake, shooter, feeder. Rumbles when auto-aligning and shooter is at target.
 */
public class OI {
    private final CommandXboxController driverJoystick;
    private final CommandXboxController operatorJoystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final DriverAssist driverAssist;
    private final Telemetry logger;

    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.FieldCentric wobbleDrive;

    public OI(CommandSwerveDrivetrain drivetrain, DriverAssist driverAssist, Telemetry logger, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.driverAssist = driverAssist;
        this.logger = logger;
        this.driverJoystick = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
        this.operatorJoystick = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

        this.drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DRIVE_DEADBAND)
                .withRotationalDeadband(Constants.ROTATION_DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.wobbleDrive = new SwerveRequest.FieldCentric()
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
                double leftX = MathUtil.applyDeadband(-driverJoystick.getLeftX(), Constants.JOYSTICK_DEADBAND);
                double rightX = MathUtil.applyDeadband(-driverJoystick.getHID().getRawAxis(Constants.ROTATION_AXIS), Constants.JOYSTICK_DEADBAND);
                double vx = leftY * Constants.MAX_SPEED;
                double vy = leftX * Constants.MAX_SPEED;
                double omega = rightX * Constants.MAX_ANGULAR_RATE;
                return drive.withVelocityX(-vx).withVelocityY(-vy).withRotationalRate(-omega);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    private void configureDrivetrainControls() {
        // LB = reset field-centric heading
        triggerDriverButton(Constants.LEFT_BUTTON).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // X = basket wobble + driver joystick rumbles
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
        operatorJoystick.leftTrigger().whileTrue(
            Commands.runEnd(shooter::runIntake, shooter::stop, shooter)
        );

        // RT = shoot wheel, Y = feeder (when target visible).
        // Operator joystick rumbles when auto-aligning (driver RB) and shooter is at target RPS.
        operatorJoystick.rightTrigger().or(operatorJoystick.y()).whileTrue(
            Commands.runEnd(
                () -> {
                    double limelightRps = driverAssist.getShooterTargetRPSFromLimelight();
                    double targetRps = limelightRps > 0 ? limelightRps : Constants.Shooter.SHOOTER_DEFAULT_RPS;
                    boolean rt = operatorJoystick.rightTrigger().getAsBoolean();
                    boolean y = operatorJoystick.y().getAsBoolean();
                    if (rt) {
                        shooter.runShoot(targetRps);
                    } else {
                        shooter.stop();
                    }
                    if (y && limelightRps > 0) {
                        shooter.runFeed();
                        operatorJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    } else {
                        shooter.stopFeeder();
                        if (rt) {
                            double actual = shooter.getShooterVelocity();
                            boolean atTarget = Math.abs(actual - targetRps) <= Constants.Shooter.SHOOTER_RPS_RUMBLE_TOLERANCE;
                            operatorJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble,
                                    atTarget ? Constants.Shooter.SHOOTER_RUMBLE_STRENGTH : 0);
                        } else {
                            operatorJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                        }
                    }
                },
                () -> {
                    shooter.stop();
                    operatorJoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                },
                shooter
            )
        );

        operatorJoystick.b().whileTrue(
            Commands.runEnd(shooter::runFeedReverse, shooter::stop, shooter)
        );

        operatorJoystick.a().whileTrue(
            Commands.runEnd(shooter::runSystemReverse, shooter::stop, shooter)
        );
    }

    private void configureDriverAssist() {
        // RB = driver assist (AprilTag aim)
        triggerDriverButton(Constants.RIGHT_BUTTON).whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(Constants.LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                driverAssist.resetAimPid();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double leftY = MathUtil.applyDeadband(driverJoystick.getLeftY(), Constants.JOYSTICK_DEADBAND);
                    double leftX = MathUtil.applyDeadband(-driverJoystick.getLeftX(), Constants.JOYSTICK_DEADBAND);
                    double vx = leftY * Constants.MAX_SPEED;
                    double vy = leftX * Constants.MAX_SPEED;
                    double omegaRadS = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
                    double turnCmd = driverAssist.calculateAimCorrection(omegaRadS);
                    turnCmd = MathUtil.clamp(turnCmd, -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);

                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(-turnCmd);
                })
            )
        );
    }

    /** Trigger for a raw button on the driver controller. */
    private Trigger triggerDriverButton(int buttonId) {
        return new Trigger(() -> driverJoystick.getHID().getRawButton(buttonId));
    }
}
