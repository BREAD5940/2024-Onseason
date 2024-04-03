package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.FenderShotCommand;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class OPAuto extends SequentialCommandGroup {

  public OPAuto(Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        // Fire preload
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.OPAutoA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.OPAutoA, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new TrajectoryFollowerCommand(
                () -> selectFirstPath(),
                swerve,
                () -> superstructure.hasPiece(),
                new ChassisSpeeds(-2, 0, 0))
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.OPAutoC, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
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
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.OPAutoD, swerve, () -> superstructure.hasPiece())
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new TrajectoryFollowerCommand(
                () -> selectThirdPath(),
                swerve,
                () -> superstructure.hasPiece(),
                new ChassisSpeeds(-2, 0, 0))
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                })
            .deadlineWith(new RunCommand(() -> shooter.requestVisionSpeaker(false))),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }

  public PathPlannerPath selectFirstPath() {
    int target = RobotContainer.visionSupplier.getTargetNote(4, 3);
    if (target == 4) {
      return Robot.OPAutoBA;
    } else {
      return Robot.OPAutoBB;
    }
  }

  public PathPlannerPath selectSecondPath() {
    int target = RobotContainer.visionSupplier.getTargetNote(3, 2);
    if (target == 3) {
      return Robot.OPAutoCA;
    } else {
      return Robot.OPAutoCB;
    }
  }

  public PathPlannerPath selectThirdPath() {
    int target = RobotContainer.visionSupplier.getTargetNote(2, 1);
    if (target == 2) {
      return Robot.OPAutoDA;
    } else {
      return Robot.OPAutoDB;
    }
  }
}
