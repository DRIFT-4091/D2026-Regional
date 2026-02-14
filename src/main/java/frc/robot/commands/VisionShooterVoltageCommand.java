package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class VisionShooterVoltageCommand extends Command{

  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final int pipeline;

  public VisionShooterVoltageCommand(Shooter shooter2, CommandSwerveDrivetrain drivetrain, int pipeline) {
    this.shooter = shooter2;
    this.drivetrain = drivetrain;
    this.pipeline = pipeline;

    // This command controls the shooter subsystem (not drivetrain)
    addRequirements(shooter2);
  }

@Override
  public void initialize() {
    Limelight.setPipeline(pipeline);
    Limelight.setLedMode(0);
  }

  @Override
  public void execute() {
    // Run your vision-assisted shooting behavior:
    // (For now: spin the same shoot/feed combo while vision pipeline is active)
    shooter.runShootFeedCombo();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs while button held
  }
}

