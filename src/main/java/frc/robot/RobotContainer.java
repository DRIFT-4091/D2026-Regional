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
import frc.robot.vision.MegaTag;

/**
 * RobotContainer - Main robot configuration and initialization.
 *
 * This class coordinates the robot's subsystems and operator interface.
 * All button bindings are configured in the OI class.
 * All constants are defined in the Constants class.
 * Driver assist features are handled by the DriverAssist class.
 * SysId dashboard wiring is in SysIdDashboard.
 */
public class RobotContainer {
    // Subsystems
    private final CommandSwerveDrivetrain drivetrain;

    private final Shooter shooter;

    // Helper classes
    private final DriverAssist driverAssist;
    private final Telemetry logger;
    private final OI oi;

    public RobotContainer() {
        // Initialize subsystems
        drivetrain = TunerConstants.createDrivetrain();
        shooter = new Shooter();

        // Initialize helper classes
        driverAssist = new DriverAssist();
        logger = new Telemetry(drivetrain, new MegaTag(drivetrain), driverAssist);

        // Initialize operator interface (configures all bindings)
        oi = new OI(drivetrain, driverAssist, logger, shooter);

        // SysId chooser and test commands on Shuffleboard
        new SysIdDashboard(drivetrain, shooter);
    }

    /**
     * Configures PathPlanner AutoBuilder for autonomous routines.
     */
    /**
     * Returns the autonomous command to run, or null if none.
     */
    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("NewAuto");
    }

    /**
     * Returns a command that snaps the drivetrain wheels to forward with bevel gears facing
     * inward. Scheduled when the robot is enabled (teleop, auto, test) so wheels align on enable.
     */
    public Command getSnapWheelsAtEnableCommand() {
        return drivetrain.snapWheelsToForwardCommand();
    }
}
