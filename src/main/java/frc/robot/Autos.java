package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.assist.DriverAssist;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

/**
 * Factory methods for autonomous routines.
 *
 * Align + Shoot (stationary):
 *   Rotates in place using Limelight aim correction until aimed at a hub tag (or 3 s timeout),
 *   pre-spinning the shooter to default RPS during alignment. Then shoots for 3 s, shakes for
 *   2 s, shoots for 3 s, shakes for 2 s, then shoots until autonomous ends.
 */
public final class Autos {
    private Autos() {}

    // -------------------------------------------------------------------------
    // Align + Shoot (stationary)
    // -------------------------------------------------------------------------

    /**
     * Rotates in place using Limelight aim correction until aimed at a hub tag (or 3 s timeout),
     * then: shoots for 3 s, shakes for 2 s, shoots for 3 s, shakes for 2 s, shoots until auto ends.
     * The shooter pre-spins during alignment so it is at speed when shooting begins.
     */
    public static Command simpleAlignAndShoot(
            CommandSwerveDrivetrain drivetrain, Shooter shooter, DriverAssist driverAssist) {

        SwerveRequest.RobotCentric holdDrive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(0)
                .withRotationalDeadband(0);

        SwerveRequest.RobotCentric wobbleDrive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(0)
                .withRotationalDeadband(0);

        // Phase 1: spin in place to find and align with hub (max 3 s), shooter pre-spins to default RPS
        Command align = Commands.parallel(
            drivetrain.applyRequest(() -> {
                double omega = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
                double turn  = MathUtil.clamp(
                        driverAssist.calculateAimCorrection(omega),
                        -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
                return holdDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(turn);
            })
            .until(() -> driverAssist.hasAnyAllianceTarget() && driverAssist.isAimed()),
            Commands.run(() -> shooter.runShoot(Constants.Shooter.SHOOTER_DEFAULT_RPS), shooter)
        ).withTimeout(3.0);

        // Helper lambda to build a shoot command for a given duration
        java.util.function.Function<Double, Command> makeShoot = (duration) -> Commands.parallel(
            drivetrain.applyRequest(() -> {
                double omega = drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond;
                double turn  = MathUtil.clamp(
                        driverAssist.calculateAimCorrection(omega),
                        -Constants.MAX_AIM_RAD_PER_SEC, Constants.MAX_AIM_RAD_PER_SEC);
                return holdDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(turn);
            }),
            Commands.runEnd(
                () -> {
                    double rps = driverAssist.getShooterTargetRPSFromLimelight();
                    double target = rps > 0 ? rps : Constants.Shooter.SHOOTER_DEFAULT_RPS;
                    shooter.runShoot(target);
                    if (shooter.getShooterVelocity() >= target - Constants.Shooter.SHOOTER_RPS_RUMBLE_TOLERANCE) {
                        shooter.runFeed();
                    }
                },
                shooter::stop,
                shooter
            )
        ).withTimeout(duration);
        // No onlyIf guard: fallback to SHOOTER_DEFAULT_RPS when no tag visible ensures shooting
        // always happens. onlyIf caused all shoot phases to be skipped when the Limelight briefly
        // lost the hub tag or rawfiducials was empty at the moment each phase started.

        // Helper lambda to build a wobble command for a given duration
        // Keeps shooter spinning at default RPS so the wheel stays warm between shoot phases.
        java.util.function.Function<Double, Command> makeWobble = (duration) ->
            Commands.parallel(
                drivetrain.applyRequest(() -> {
                    double t = Timer.getFPGATimestamp();
                    double vx = Constants.WOBBLE_SPEED_MPS * Math.sin(2 * Math.PI * Constants.WOBBLE_HZ * t);
                    return wobbleDrive.withVelocityX(vx).withVelocityY(0).withRotationalRate(0);
                }),
                Commands.run(() -> shooter.runShoot(Constants.Shooter.SHOOTER_DEFAULT_RPS), shooter)
            ).withTimeout(duration);

        return Commands.sequence(
            Commands.runOnce(() -> {
                Limelight.setPipeline(Constants.LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                driverAssist.resetAimPid();
            }),
            align,
            Commands.runOnce(driverAssist::resetAimPid),  // clear alignmentTimedOut before shooting
            makeShoot.apply(3.0),   // shoot 3 s
            makeWobble.apply(2.0),  // shake 2 s
            Commands.runOnce(driverAssist::resetAimPid),  // re-arm aim for second shoot phase
            makeShoot.apply(3.0),   // shoot 3 s
            makeWobble.apply(2.0),  // shake 2 s
            Commands.runOnce(driverAssist::resetAimPid),  // re-arm aim for final shoot phase
            makeShoot.apply(Double.MAX_VALUE)    // shoot until autonomous ends
        );
    }
}
