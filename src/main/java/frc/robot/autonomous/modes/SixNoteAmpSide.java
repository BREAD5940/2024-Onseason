package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class SixNoteAmpSide extends SequentialCommandGroup {

  public SixNoteAmpSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("SIX_NOTE_AMP_SIDE");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.sixNoteAmpSideA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new InstantCommand(
            () -> {
              shooter.requestVisionSpeaker(false);
              superstructure.requestVisionSpeaker(true, true, false);
            }),
        new TrajectoryFollowerCommand(() -> Robot.sixNoteAmpSideA, swerve, false, () -> true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, true, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new TrajectoryFollowerCommand(() -> Robot.sixNoteAmpSideB, swerve, false, () -> true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, true, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new TrajectoryFollowerCommand(
                () -> Robot.sixNoteAmpSideC, swerve, false, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, true, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false)))
            .alongWith(
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () ->
                            RobotContainer.visionSupplier.getDistance()
                                > Units.inchesToMeters(200.0)),
                    new InstantCommand(
                        () -> superstructure.requestVisionSpeaker(true, false, false)))),
        new StationaryShootCommand(swerve, superstructure, shooter, 0, 0),
        new TrajectoryFollowerCommand(
                () -> Robot.sixNoteAmpSideD, swerve, false, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, false, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(1.0));
  }
}
