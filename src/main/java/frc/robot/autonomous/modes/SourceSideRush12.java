package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class SourceSideRush12 extends SequentialCommandGroup {

  public SourceSideRush12(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("SOURCE_SIDE_RUSH_12");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.ssrRushA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
              superstructure.unregisterAutoPreload();
            }),
        new TrajectoryFollowerCommand(() -> Robot.ssrRushA, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                }),
        new WaitUntilCommand(() -> superstructure.hasPiece() || intake.hasPiece()).withTimeout(0.5),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(() -> Robot.ssrReturnA, swerve, false, () -> true)
                    .deadlineWith(
                        new RunCommand(
                            () -> {
                              shooter.requestVisionSpeaker(false);
                              superstructure.requestVisionSpeaker(true, false, false);
                            })),
                new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3),
                new TrajectoryFollowerCommand(() -> Robot.ssrFromShootPoseB, swerve, () -> false),
                new WaitUntilCommand(() -> superstructure.hasPiece() || intake.hasPiece())
                    .withTimeout(0.3),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnB, swerve, false, () -> true)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3)),
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                            () -> Robot.ssrPivotC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3)),
                    () -> superstructure.hasPiece() || intake.hasPiece())),
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(() -> Robot.ssrPivotB, swerve, false, () -> false),
                new WaitUntilCommand(() -> superstructure.hasPiece() || intake.hasPiece())
                    .withTimeout(0.3),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnB, swerve, false, () -> true)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3),
                        new TrajectoryFollowerCommand(
                            () -> Robot.ssrFromShootPoseC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3)),
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                            () -> Robot.ssrPivotC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter, -0.15, -0.3)),
                    () -> superstructure.hasPiece() || intake.hasPiece())),
            () -> superstructure.hasPiece() || intake.hasPiece()));
  }
}
