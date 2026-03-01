package frc.robot;

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

        logger = new Telemetry(Constants.MAX_SPEED);
        driverAssist = new DriverAssist();

        // Initialize operator interface (configures all bindings)
        oi = new OI(drivetrain, driverAssist, logger, shooter);
    }

    /**
     * Returns the autonomous command to run, or null if none.
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
