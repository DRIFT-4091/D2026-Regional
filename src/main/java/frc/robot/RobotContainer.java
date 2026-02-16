package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/**
 * RobotContainer - Main robot configuration and initialization.
 * 
 * This class coordinates the robot's subsystems and operator interface.
 * All button bindings are configured in the OI class.
 * All constants are defined in the Constants class.
 * Driver assist features are handled by the DriverAssist class.
 */
public class RobotContainer {
    // Subsystems
    private final CommandSwerveDrivetrain drivetrain;
    
    //private final Shooter shooter;

    // Helper classes
    private final DriverAssist driverAssist;
    private final Telemetry logger;
    private final OI oi;

    public RobotContainer() {
        // Initialize subsystems
        drivetrain = TunerConstants.createDrivetrain();

        // No shooter because we are practicing

            // shooter = new Shooter();

        // Initialize helper classes

        logger = new Telemetry(Constants.MAX_SPEED);
        driverAssist = new DriverAssist();

        // Configure PathPlanner auto
        configurePathPlanner();

        // Initialize operator interface (configures all bindings)
        oi = new OI(drivetrain, driverAssist, logger);
    }

    /**
     * Configures PathPlanner AutoBuilder for autonomous routines.
     */
    private void configurePathPlanner() {
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load PathPlanner GUI settings", e);
        }

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::getRobotRelativeSpeeds,
            drivetrain::driveRobotRelative,

            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
            ),

            config,

            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red,

            (edu.wpi.first.wpilibj2.command.Subsystem) drivetrain
        );
    }

    /**
     * Returns the autonomous command to run.
     */
    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("2026V1");
    }
}
