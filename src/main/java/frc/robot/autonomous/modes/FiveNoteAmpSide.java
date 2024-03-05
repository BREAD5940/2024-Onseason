package frc.robot.autonomous.modes;

import static frc.robot.constants.Constants.AutoShots.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.AimAtSpeakerCommand;
import frc.robot.subsystems.commands.FenderShotCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class FiveNoteAmpSide extends SequentialCommandGroup {

  public FiveNoteAmpSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        // Fire preload
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.fiveNoteMidlineAmpSideA;
              if (DriverStation.getAlliance().get() == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter).withTimeout(2),

        // Intake first piece and shoot
        new InstantCommand(
            () ->
                shooter.requestSpeakerOverride(
                    FIVE_NOTE_AMP_FIRST_SHOT.leftRPM, FIVE_NOTE_AMP_FIRST_SHOT.rightRPM)),
        new TrajectoryFollowerCommand(Robot.fiveNoteAmpSideA, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new InstantCommand(
            () -> {
              intake.requestIdle();
              superstructure.requestVisionSpeaker(
                  true,
                  true,
                  false,
                  FIVE_NOTE_AMP_FIRST_SHOT.pivotAngleDeg,
                  FIVE_NOTE_AMP_FIRST_SHOT.elevatorHeight);
            }),
        new WaitUntilCommand(() -> !superstructure.hasPiece())
            .withTimeout(3.0)
            .deadlineWith(new AimAtSpeakerCommand(swerve)),

        // Intake second piece and shoot
        new InstantCommand(
            () ->
                shooter.requestSpeakerOverride(
                    FIVE_NOTE_AMP_SECOND_SHOT.leftRPM, FIVE_NOTE_AMP_SECOND_SHOT.rightRPM)),
        new TrajectoryFollowerCommand(Robot.fiveNoteAmpSideB, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new InstantCommand(
            () -> {
              intake.requestIdle();
              superstructure.requestVisionSpeaker(
                  true,
                  true,
                  false,
                  FIVE_NOTE_AMP_SECOND_SHOT.pivotAngleDeg,
                  FIVE_NOTE_AMP_SECOND_SHOT.elevatorHeight);
            }),
        new WaitUntilCommand(() -> !superstructure.hasPiece())
            .withTimeout(3.0)
            .deadlineWith(new AimAtSpeakerCommand(swerve)),

        // Intake third piece and shoot
        new InstantCommand(
            () ->
                shooter.requestSpeakerOverride(
                    FIVE_NOTE_AMP_THIRD_SHOT.leftRPM, FIVE_NOTE_AMP_THIRD_SHOT.rightRPM)),
        new TrajectoryFollowerCommand(Robot.fiveNoteAmpSideC, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new InstantCommand(
            () -> {
              intake.requestIdle();
              superstructure.requestVisionSpeaker(
                  true,
                  true,
                  false,
                  FIVE_NOTE_AMP_THIRD_SHOT.pivotAngleDeg,
                  FIVE_NOTE_AMP_THIRD_SHOT.elevatorHeight);
            }),
        new WaitUntilCommand(() -> !superstructure.hasPiece())
            .withTimeout(3.0)
            .deadlineWith(new AimAtSpeakerCommand(swerve)),

        // Intake fourth piece and shoot
        new InstantCommand(
            () ->
                shooter.requestSpeakerOverride(
                    FIVE_NOTE_AMP_FOURTH_SHOT.leftRPM, FIVE_NOTE_AMP_FOURTH_SHOT.rightRPM)),
        new TrajectoryFollowerCommand(Robot.fiveNoteAmpSideD, swerve, true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new InstantCommand(
            () -> {
              intake.requestIdle();
              superstructure.requestVisionSpeaker(
                  true,
                  true,
                  false,
                  FIVE_NOTE_AMP_FOURTH_SHOT.pivotAngleDeg,
                  FIVE_NOTE_AMP_FOURTH_SHOT.elevatorHeight);
            }),
        new WaitUntilCommand(() -> !superstructure.hasPiece())
            .withTimeout(3.0)
            .deadlineWith(new AimAtSpeakerCommand(swerve)));
  }
}
