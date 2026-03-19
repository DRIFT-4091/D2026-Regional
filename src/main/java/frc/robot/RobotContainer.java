package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.assist.AlignTarget;
import frc.robot.assist.DriverAssist;
import frc.robot.dashboard.GameData;
import frc.robot.dashboard.SysIdDashboard;
import frc.robot.dashboard.Telemetry;
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
    private final GameData gameData;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<AlignTarget> alignTargetChooser;

    public RobotContainer() {
        // Initialize subsystems
        drivetrain = TunerConstants.createDrivetrain();
        shooter = new Shooter();

        // Alignment target chooser (Hub / Trench / Human Player) — shown on Elastic
        alignTargetChooser = new SendableChooser<>();
        alignTargetChooser.setDefaultOption(AlignTarget.HUB.label, AlignTarget.HUB);
        alignTargetChooser.addOption(AlignTarget.TRENCH.label, AlignTarget.TRENCH);
        alignTargetChooser.addOption(AlignTarget.HUMAN_PLAYER.label, AlignTarget.HUMAN_PLAYER);
        SmartDashboard.putData("Align Target", alignTargetChooser);

        // Initialize helper classes
        MegaTag megaTag = new MegaTag(drivetrain);
        driverAssist = new DriverAssist(alignTargetChooser);
        logger = new Telemetry(drivetrain, megaTag, driverAssist);
        gameData = new GameData();

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Align + Shoot", Autos.simpleAlignAndShoot(drivetrain, shooter, driverAssist));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Initialize operator interface (configures all bindings)
        new OI(drivetrain, driverAssist, logger, shooter);

        // SysId chooser and test commands on Shuffleboard
        new SysIdDashboard(drivetrain, shooter);

        // LifeCam stream - Elastic auto-discovers CameraServer streams
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(320, 240);
        camera.setFPS(15);
    }

    /** Called every robot loop to push FMS game data to NetworkTables. */
    public void periodic() {
        gameData.update();
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
