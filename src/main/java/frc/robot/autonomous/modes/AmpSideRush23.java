package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class AmpSideRush23 extends SequentialCommandGroup {

  public AmpSideRush23(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("AMP_SIDE_RUSH_23");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestFender(true, true);
              shooter.requestFender();

              PathPlannerPath path = Robot.asrRushB;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.asrRushB, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                }),
        new TrajectoryFollowerCommand(() -> Robot.asrReturnB, swerve, false, () -> true)
            .deadlineWith(
                new RunCommand(
                    () -> {
                      shooter.requestVisionSpeaker(false);
                      superstructure.requestVisionSpeaker(true, false, false);
                    })),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.asrFromShootPoseC, swerve, () -> false),
        new TrajectoryFollowerCommand(() -> Robot.asrReturnC, swerve, () -> false)
            .deadlineWith(
                new RunCommand(
                    () -> {
                      shooter.requestVisionSpeaker(false);
                      superstructure.requestVisionSpeaker(true, false, false);
                    })),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }
}
