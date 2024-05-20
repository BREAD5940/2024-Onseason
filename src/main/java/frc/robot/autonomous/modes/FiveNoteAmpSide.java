package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class FiveNoteAmpSide extends SequentialCommandGroup {

  public FiveNoteAmpSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("FIVE_NOTE_AMP_SIDE");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        // Fire preload
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.fiveNoteAmpSideA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter).withTimeout(3),
        new TrajectoryFollowerCommand(() -> Robot.fiveNoteAmpSideA, swerve, () -> true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(() -> Robot.fiveNoteAmpSideB, swerve, () -> true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(() -> Robot.fiveNoteAmpSideC, swerve, () -> true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(() -> Robot.fiveNoteAmpSideD, swerve, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }

  private PathPlannerPath selectPath() {
    int target = RobotContainer.visionSupplier.getTargetNote(4, 3);
    if (target == 4) {
      return Robot.fiveNoteAmpSideEA;
    } else {
      return Robot.fiveNoteAmpSideEB;
    }
  }
}
