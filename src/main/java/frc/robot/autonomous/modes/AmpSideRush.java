package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class AmpSideRush extends SequentialCommandGroup {

  public AmpSideRush(Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("AMP_SIDE_RUSH");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestFender(true, true);
              shooter.requestFender();

              PathPlannerPath path = Robot.asrRushA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.asrRushA, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece() || intake.hasPiece()).withTimeout(0.5),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(() -> Robot.asrReturnA, swerve, false, () -> true)
                    .deadlineWith(
                        new RunCommand(
                            () -> {
                              shooter.requestVisionSpeaker(false);
                              superstructure.requestVisionSpeaker(true, false, false);
                            })),
                new StationaryShootCommand(swerve, superstructure, shooter),
                new TrajectoryFollowerCommand(() -> Robot.asrFromShootPoseB, swerve, () -> false),
                new WaitUntilCommand(() -> superstructure.hasPiece() || intake.hasPiece())
                    .withTimeout(0.5),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                                () -> Robot.asrReturnB, swerve, false, () -> true)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter),
                        new TrajectoryFollowerCommand(
                            () -> Robot.asrFromShootPoseC, swerve, () -> false),
                        new TrajectoryFollowerCommand(() -> Robot.asrReturnC, swerve, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                            () -> Robot.asrPivotC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.asrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    () -> superstructure.hasPiece() || intake.hasPiece())),
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(() -> Robot.asrPivotB, swerve, false, () -> false),
                new WaitUntilCommand(() -> superstructure.hasPiece() || intake.hasPiece())
                    .withTimeout(0.5),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                                () -> Robot.asrReturnB, swerve, false, () -> true)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter),
                        new TrajectoryFollowerCommand(
                            () -> Robot.asrFromShootPoseC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.asrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                            () -> Robot.asrPivotC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.asrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    () -> superstructure.hasPiece() || intake.hasPiece())),
            () -> superstructure.hasPiece() || intake.hasPiece()));
  }
}
