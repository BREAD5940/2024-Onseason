package frc.robot.subsystems.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class FenderShotCommand extends Command {
  // Subsystems
  private Swerve swerve;
  private Superstructure superstructure;
  private Shooter shooter;

  public FenderShotCommand(Swerve swerve, Superstructure superstructure, Shooter shooter) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.shooter = shooter;

    addRequirements(swerve, superstructure, shooter);
  }

  @Override
  public void execute() {
    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), true);

    shooter.requestFender();
    superstructure.requestFender(true, true);
  }

  @Override
  public boolean isFinished() {
    return !superstructure.hasPiece()
        && superstructure.getSystemState() == SuperstructureState.IDLE;
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.requestIdle();
    superstructure.requestFender(false, false);
  }
}
