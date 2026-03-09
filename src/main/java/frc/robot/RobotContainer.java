package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Initialize subsystems
        drivetrain = TunerConstants.createDrivetrain();
        shooter = new Shooter();

        // Initialize helper classes
        driverAssist = new DriverAssist();
        logger = new Telemetry(drivetrain, new MegaTag(drivetrain), driverAssist);

        // Configure PathPlanner auto (must run before building chooser)
        configurePathPlanner();

        // Build auto chooser from all .auto files, then add vision-based routines
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Hub Score", Autos.scoreAtHub(drivetrain, shooter, driverAssist));
        autoChooser.addOption("Hub Score + Return", Autos.scoreAndReturn(drivetrain, shooter, driverAssist));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Initialize operator interface (configures all bindings)
        oi = new OI(drivetrain, driverAssist, logger, shooter);

        // SysId chooser and test commands on Shuffleboard
        new SysIdDashboard(drivetrain, shooter);

        // LifeCam stream - Elastic auto-discovers CameraServer streams
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(320, 240);
        camera.setFPS(30);
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
            (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds, feedforwards),

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
     * Returns the autonomous command to run, or null if none.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Returns a command that snaps the drivetrain wheels to forward with bevel gears facing
     * inward. Scheduled when the robot is enabled (teleop, auto, test) so wheels align on enable.
     */
    public Command getSnapWheelsAtEnableCommand() {
        return drivetrain.snapWheelsToForwardCommand();
    }
}
