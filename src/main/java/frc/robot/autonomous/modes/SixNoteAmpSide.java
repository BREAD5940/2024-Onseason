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

public class SixNoteAmpSide extends SequentialCommandGroup {

  public SixNoteAmpSide(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(1.0),
        new InstantCommand(() -> superstructure.requestIntake(false)),
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.sixNoteAmpSideA;
              if (DriverStation.getAlliance().get() == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter).withTimeout(2.0),
        new InstantCommand(() -> shooter.requestVisionSpeaker(false)),
        new TrajectoryFollowerCommand(Robot.sixNoteAmpSideA, swerve, false, () -> true)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  shooter.requestVisionSpeaker(false);
                  superstructure.requestVisionSpeaker(true, true, false);
                }),
        new WaitUntilCommand(() -> !superstructure.hasPiece()).withTimeout(1),
        new TrajectoryFollowerCommand(
                Robot.sixNoteAmpSideB, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(
                Robot.sixNoteAmpSideC, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(true, false, false);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2));
  }
}
