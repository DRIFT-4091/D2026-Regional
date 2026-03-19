package frc.robot.dashboard;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/**
 * SysId dashboard wiring: chooser and four test commands on Shuffleboard.
 * Subsystems (drivetrain, shooter) own the actual SysIdRoutines; this class only
 * publishes the UI and defers to the selected routine at schedule time.
 */
public class SysIdDashboard {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;

    private final SendableChooser<String> sysIdChooser = new SendableChooser<>();

    public SysIdDashboard(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        configure();
    }

    /**
     * Puts SysId routine chooser and four test commands on Shuffleboard.
     * Each command reads the chooser at schedule time and runs the selected routine.
     */
    private void configure() {
        sysIdChooser.setDefaultOption("Translation (Drive)", "translation");
        sysIdChooser.addOption("Steer", "steer");
        sysIdChooser.addOption("Rotation", "rotation");
        sysIdChooser.addOption("Shooter", "shooter");
        SmartDashboard.putData("SysId/Routine", sysIdChooser);

        SmartDashboard.putData("SysId/Quasistatic Forward", buildSysIdCommand(true, true));
        SmartDashboard.putData("SysId/Quasistatic Reverse", buildSysIdCommand(true, false));
        SmartDashboard.putData("SysId/Dynamic Forward", buildSysIdCommand(false, true));
        SmartDashboard.putData("SysId/Dynamic Reverse", buildSysIdCommand(false, false));
    }

    /**
     * Builds a command that runs the selected SysId test. Chooser is read when the command is
     * scheduled (defer), so the dashboard selection is used at press time.
     */
    private Command buildSysIdCommand(boolean quasistatic, boolean forward) {
        return Commands.defer(
                () -> {
                    String sel = sysIdChooser.getSelected();
                    if (sel == null) sel = "translation";
                    Direction d = forward ? Direction.kForward : Direction.kReverse;
                    switch (sel) {
                        case "steer":
                            return quasistatic
                                    ? drivetrain.sysIdSteerQuasistatic(d)
                                    : drivetrain.sysIdSteerDynamic(d);
                        case "rotation":
                            return quasistatic
                                    ? drivetrain.sysIdRotationQuasistatic(d)
                                    : drivetrain.sysIdRotationDynamic(d);
                        case "shooter":
                            return quasistatic
                                    ? shooter.sysIdQuasistatic(d)
                                    : shooter.sysIdDynamic(d);
                        default:
                            return quasistatic
                                    ? drivetrain.sysIdTranslationQuasistatic(d)
                                    : drivetrain.sysIdTranslationDynamic(d);
                    }
                },
                Set.of(drivetrain, shooter));
    }
}
