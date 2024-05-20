package frc.robot.autonomous.modes;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class DriveBackTwo extends SequentialCommandGroup {

  public DriveBackTwo(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("DRIVE_BACK_TWO");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        // Fire preload
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new StationaryShootCommand(swerve, superstructure, shooter),

        // Drive back and shoot
        new InstantCommand(() -> swerve.requestVelocity(new ChassisSpeeds(-1.0, 0.0, 0.0), false))
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(3.0),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new InstantCommand(
            () -> {
              swerve.requestVelocity(new ChassisSpeeds(0.0, 0.0, 0.0), false);
              intake.requestIdle();
              shooter.requestIdle();
            }));
  }
}
