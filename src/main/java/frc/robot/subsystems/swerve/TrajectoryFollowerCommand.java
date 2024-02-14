package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadHolonomicDriveController;
import org.littletonrobotics.junction.Logger;

public class TrajectoryFollowerCommand extends Command {

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;
  private final boolean isInitialPoint;
  private final Swerve swerve;
  private final Timer timer = new Timer();

  public final BreadHolonomicDriveController autonomusController =
      new BreadHolonomicDriveController(
          new PIDController(4.0, 0, 0.0),
          new PIDController(4.0, 0, 0.0),
          new PIDController(4.0, 0, 0.0));

  public TrajectoryFollowerCommand(PathPlannerPath path, Swerve swerve, boolean isInitialPoint) {
    this.path = path;
    this.swerve = swerve;
    this.isInitialPoint = isInitialPoint;
    addRequirements(swerve);
  }

  public TrajectoryFollowerCommand(PathPlannerPath path, Swerve swerve) {
    this(path, swerve, false);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("TrajectoryFollowerCommandAlliance", DriverStation.getAlliance().get());
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      path = path.flipPath();
    }
    if (isInitialPoint) {
      trajectory =
          path.getTrajectory(
              new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());
      swerve.resetPose(path.getPreviewStartingHolonomicPose());
    } else {
      trajectory =
          path.getTrajectory(new ChassisSpeeds(), RobotContainer.swerve.getPose().getRotation());
    }
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    PathPlannerTrajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds adjustedSpeeds =
        autonomusController.calculate(RobotContainer.swerve.getPose(), goal);
    swerve.requestVelocity(adjustedSpeeds, false);

    Logger.recordOutput(
        "TrajectoryFollower/TrajectoryGoal",
        new Pose2d(goal.positionMeters, goal.targetHolonomicRotation));
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false);
  }
}
