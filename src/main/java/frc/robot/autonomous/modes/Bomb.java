package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.FenderShotCommand;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class Bomb extends SequentialCommandGroup {

  public Bomb(Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.bombA;
              if (DriverStation.getAlliance().get() == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter).withTimeout(3),
        new TrajectoryFollowerCommand(Robot.bombA, swerve, false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(
                new SequentialCommandGroup(
                    new WaitCommand(3.43),
                    new InstantCommand(() -> shooter.requestVisionSpeaker(true)))),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.bombB, swerve, false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(
                new SequentialCommandGroup(
                    new WaitCommand(2.17),
                    new InstantCommand(() -> shooter.requestVisionSpeaker(true)))),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.bombC, swerve, false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(
                new SequentialCommandGroup(
                    new WaitCommand(3.2),
                    new InstantCommand(() -> shooter.requestVisionSpeaker(true)))),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2));
  }
}
