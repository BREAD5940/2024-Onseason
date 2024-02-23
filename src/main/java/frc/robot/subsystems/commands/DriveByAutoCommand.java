package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.shooter.Shooter;

public class DriveByAutoCommand extends Command {
  private Superstructure superstructure;
  private Shooter shooter;
  private boolean visionSpeakerRequested;

  private double startTime;

  public DriveByAutoCommand(Superstructure superstructure, Shooter shooter) {
    this.superstructure = superstructure;
    this.shooter = shooter;
    this.visionSpeakerRequested = false;

    addRequirements(superstructure, shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    shooter.requestAutoDriveBy();

    // if (!visionSpeakerRequested && Timer.getFPGATimestamp() - startTime >= 1.0) {
    superstructure.requestVisionSpeaker(true, true, false, true);
    //   visionSpeakerRequested = true;
    // }
  }

  @Override
  public boolean isFinished() {
    return !superstructure.hasPiece()
        && superstructure.getSystemState() == SuperstructureState.IDLE;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle();
    superstructure.requestVisionSpeaker(false, false, false, false);
  }
}
