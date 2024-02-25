package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class FourNoteSourceSide extends SequentialCommandGroup {

  public FourNoteSourceSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.fourNoteSourceSideA;
              if (DriverStation.getAlliance().get() == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(3),
        new TrajectoryFollowerCommand(Robot.fourNoteSourceSideA, swerve, false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false, false);
                  ;
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.fourNoteSourceSideB, swerve, false),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.fourNoteSourceSideC, swerve, false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false, false);
                  ;
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.fourNoteSourceSideD, swerve, false),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.fourNoteSourceSideE, swerve, false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false, false);
                  ;
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.fourNoteSourceSideF, swerve, false),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2));
  }
}
