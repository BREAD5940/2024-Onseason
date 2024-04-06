package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class ReverseFiveNoteAmpSide extends SequentialCommandGroup {

  public ReverseFiveNoteAmpSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.reverseFiveNoteA;
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
        new TrajectoryFollowerCommand(
                () -> Robot.reverseFiveNoteA, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, true, false);
                  shooter.requestVisionSpeaker(false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new WaitUntilCommand(() -> !superstructure.hasPiece()).withTimeout(1.0),
        new TrajectoryFollowerCommand(
                () -> Robot.reverseFiveNoteB, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, true, false);
                  shooter.requestVisionSpeaker(false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new TrajectoryFollowerCommand(
                () -> selectFirstPath(),
                swerve,
                () -> superstructure.hasPiece(),
                new ChassisSpeeds(-3, 0, 0))
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, false, false);
                  shooter.requestVisionSpeaker(false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(
                () -> Robot.reverseFiveNoteD, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, false, false);
                  shooter.requestVisionSpeaker(false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new TrajectoryFollowerCommand(
                () -> selectSecondPath(),
                swerve,
                () -> superstructure.hasPiece(),
                new ChassisSpeeds(-2, 0, 0))
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, false, false);
                  shooter.requestVisionSpeaker(false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false)))
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(3.6),
                    new InstantCommand(
                        () -> superstructure.requestVisionSpeaker(true, true, false)))),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }

  public PathPlannerPath selectFirstPath() {
    int target = RobotContainer.visionSupplier.getTargetNote(4, 3);
    System.out.println("\n\n\n\n\nSelecting target with obj det: " + target);
    if (target == 4) {
      return Robot.reverseFiveNoteCB;
    } else {
      return Robot.reverseFiveNoteCA;
    }
  }

  public PathPlannerPath selectSecondPath() {
    int target = RobotContainer.visionSupplier.getTargetNote(3, 2);
    System.out.println("\n\n\n\n\nSelecting target with obj det 2: " + target);
    if (target == 3) {
      return Robot.reverseFiveNoteEB;
    } else {
      return Robot.reverseFiveNoteEA;
    }
  }
}
