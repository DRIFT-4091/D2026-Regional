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
import frc.robot.commands.VisionShooterVoltageCommand;
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
    private static final int TARGET_TAG_ID = 3;

    private final PIDController aimPid = new PIDController(0.06, 0.0, 0.001);
    private final PIDController distancePid = new PIDController(0.6, 0.0, 0.0);

    private static final double DESIRED_TA = 2.0;
    private static final double READY_TX_DEG = 1.0;
    private static final double READY_TA_TOL = 0.15;

    private static final double MAX_AIM_RAD_PER_SEC = 2.5;
    private static final double MAX_AUTO_FWD_MPS = 2.0;

    private static final double READY_RUMBLE = 1.0;

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

        // Cross = brake
        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // Circle = point wheels
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // =========================================================
        // FINAL BUTTON LOGIC (SWITCHED)
        //
        // Triangle = SHOOT/FEED combo
        // R2       = VisionShooterVoltageCommand (auto logic)
        // L2       = Intake combo
        // =========================================================

        // -------- Triangle: SHOOT/FEED COMBO (top shoots + bottom feeds) --------
        joystick.triangle().whileTrue(
            Commands.runEnd(shooter::runShootFeedCombo, shooter::stop, shooter)
        );

        // -------- R2: VISION SHOOTER LOGIC (auto command) --------
        joystick.R2().whileTrue(
            new VisionShooterVoltageCommand(shooter, drivetrain, LL_AIM_PIPELINE)
        ).onFalse(
            Commands.runOnce(shooter::stop, shooter)
        );

        // -------- L2: INTAKE COMBO (bottom intake + top assist) --------
        joystick.L2().whileTrue(
            Commands.runEnd(shooter::runIntakeCombo, shooter::stop, shooter)
        );

        // SysId mappings (Share/Options + Triangle/Square)
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading on L1
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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

                    if (Limelight.hasTarget(TARGET_TAG_ID)) {
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

    private boolean isShotReady() {
        if (!Limelight.hasTarget(TARGET_TAG_ID)) return false;

        double tx = Limelight.tx();
        double ta = Limelight.ta();

        boolean aligned = Math.abs(tx) <= READY_TX_DEG;
        boolean atDistance = Math.abs(ta - DESIRED_TA) <= READY_TA_TOL;

        return aligned && atDistance;
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



