package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/**
 * Operator Interface - handles all controller bindings and driver inputs.
 */
public class OI {
    private final CommandPS4Controller joystick;
    private final CommandSwerveDrivetrain drivetrain;

    // No shooter
        //private final Shooter shooter;

    private final DriverAssist driverAssist;
    private final Telemetry logger;

    // Swerve requests
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.SwerveDriveBrake brake;
    private final SwerveRequest.PointWheelsAt point;

    public OI(CommandSwerveDrivetrain drivetrain, DriverAssist driverAssist, Telemetry logger) {
        this.drivetrain = drivetrain;
        //this.shooter = shooter;
        this.driverAssist = driverAssist;
        this.logger = logger;
        this.joystick = new CommandPS4Controller(Constants.CONTROLLER_PORT);

        // Initialize swerve requests
        this.drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DRIVE_DEADBAND)
                .withRotationalDeadband(Constants.ROTATION_DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.brake = new SwerveRequest.SwerveDriveBrake();
        this.point = new SwerveRequest.PointWheelsAt();

        configureBindings();
    }

    /**
     * Configures all controller bindings.
     */
    private void configureBindings() {
        configureDefaultCommands();
        configureDrivetrainControls();
       // configureShooterControls();
        configureSysIdControls();
        configureDriverAssist();
        configureRumbleFeedback();

        // Register telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Configures default commands.
     */
    private void configureDefaultCommands() {
        // Default manual drive
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * Constants.MAX_SPEED)
                    .withVelocityY(-joystick.getLeftX() * Constants.MAX_SPEED)
                    .withRotationalRate(-joystick.getRightX() * Constants.MAX_ANGULAR_RATE)
            )
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    /**
     * Configures drivetrain controls.
     */
    private void configureDrivetrainControls() {
        // Cross = brake wheels
        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // Circle = point wheels (manual wheel direction)
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // L1 = reset field-centric heading
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    /**
     * Configures shooter controls.
     */
    // private void configureShooterControls() {
    //     // R2 = shoot (auto voltage from Limelight distance)
    //     joystick.R2().whileTrue(
    //         Commands.runEnd(
    //             () -> shooter.runShoot(driverAssist.getShooterVoltageFromLimelight()),
    //             shooter::stop,
    //             shooter
    //         )
    //     );

    //     // L2 = intake
    //     joystick.L2().whileTrue(
    //         Commands.runEnd(shooter::runIntake, shooter::stop, shooter)
    //     );

    //     // L1 + Circle = unjam feeder (reverse)
    //     joystick.L1().and(joystick.circle()).whileTrue(
    //         Commands.runEnd(shooter::runFeedReverse, shooter::stop, shooter)
    //     );

    //     // L1 + Square = unjam intake (reverse)
    //     joystick.L1().and(joystick.square()).whileTrue(
    //         Commands.runEnd(shooter::runIntakeReverse, shooter::stop, shooter)
    //     );
    // }

    /**
     * Configures System Identification (SysId) controls.
     */
    private void configureSysIdControls() {
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    /**
     * Configures Limelight-based driver assist (aim assist).
     */
    private void configureDriverAssist() {
        joystick.R1().whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(Constants.LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                driverAssist.resetAimPid();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vx = joystick.getLeftY() * Constants.MAX_SPEED;
                    double vy = -joystick.getLeftX() * Constants.MAX_SPEED;
                    double omega = -joystick.getRightX() * Constants.MAX_ANGULAR_RATE;

                    // Auto-aim if any alliance AprilTag is visible
                    if (driverAssist.hasAnyAllianceTarget()) {
                        double turnCmd = driverAssist.calculateAimCorrection();
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

    /**
     * Configures controller rumble feedback when ready to shoot.
     */
    private void configureRumbleFeedback() {
        Trigger shotReady = new Trigger(() ->
            DriverStation.isTeleopEnabled() && driverAssist.isShotReady()
        );

        shotReady.whileTrue(
            Commands.run(() ->
                joystick.setRumble(RumbleType.kBothRumble, Constants.READY_RUMBLE)
            )
        );

        shotReady.onFalse(
            Commands.runOnce(() ->
                joystick.setRumble(RumbleType.kBothRumble, 0.0)
            )
        );
    }
}
