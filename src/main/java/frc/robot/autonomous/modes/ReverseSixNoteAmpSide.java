package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.FenderShotCommand;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class ReverseSixNoteAmpSide extends SequentialCommandGroup {

  public ReverseSixNoteAmpSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.reverseSixNoteA;
              if (DriverStation.getAlliance().get() == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter).withTimeout(2),
        new InstantCommand(() -> shooter.requestVisionSpeaker(true)),
        new TrajectoryFollowerCommand(Robot.reverseSixNoteA, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(1.0),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.reverseSixNoteB, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(1.0),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.reverseSixNoteC, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(1.0),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.reverseSixNoteD, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(1.0),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.reverseSixNoteE, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(1.0),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2));
  }
}
