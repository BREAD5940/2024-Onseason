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
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class SourceSideRush21 extends SequentialCommandGroup {

  public SourceSideRush21(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("SOURCE_SIDE_RUSH_21");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.ssrRushB;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
              superstructure.unregisterAutoPreload();
            }),
        new TrajectoryFollowerCommand(() -> Robot.ssrRushB, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                }),
        new WaitUntilCommand(superstructure::hasPiece).withTimeout(0.3),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(() -> Robot.ssrReturnB, swerve, false, () -> true)
                    .deadlineWith(
                        new RunCommand(
                            () -> {
                              shooter.requestVisionSpeaker(false);
                              superstructure.requestVisionSpeaker(true, false, false);
                            })),
                new StationaryShootCommand(swerve, superstructure, shooter),
                new TrajectoryFollowerCommand(() -> Robot.ssrFromShootPoseA, swerve, () -> false),
                new WaitUntilCommand(superstructure::hasPiece).withTimeout(0.3),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnA, swerve, false, () -> true)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                            () -> Robot.ssrSkipPivotC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    superstructure::hasPiece)),
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(() -> Robot.ssrPivotA, swerve, false, () -> false),
                new WaitUntilCommand(superstructure::hasPiece).withTimeout(0.3),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnA, swerve, false, () -> true)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter),
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
                        new StationaryShootCommand(swerve, superstructure, shooter),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(
                            () -> Robot.ssrSkipPivotC, swerve, false, () -> false),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrReturnC, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter),
                        new TrajectoryFollowerCommand(
                                () -> Robot.ssrPreloadShoot, swerve, false, () -> false)
                            .deadlineWith(
                                new RunCommand(
                                    () -> {
                                      shooter.requestVisionSpeaker(false);
                                      superstructure.requestVisionSpeaker(true, false, false);
                                    })),
                        new StationaryShootCommand(swerve, superstructure, shooter)),
                    superstructure::hasPiece)),
            superstructure::hasPiece));
  }
}
