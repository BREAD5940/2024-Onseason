package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadHolonomicDriveController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TrajectoryFollowerCommand extends Command {

  private Supplier<PathPlannerPath> pathSupplier;
  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;
  private final boolean isInitialPoint;
  private final Swerve swerve;
  private final Timer timer = new Timer();
  private boolean flippedForRed = false;
  private Supplier<Boolean> aimAtSpeaker = () -> false;
  private boolean firstTime = false;
  private ChassisSpeeds startingSpeeds;

  public final BreadHolonomicDriveController autonomusController =
      new BreadHolonomicDriveController(
          new PIDController(8.0, 0, 0.0),
          new PIDController(8.0, 0, 0.0),
          new PIDController(7.0, 0, 0.2));

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Swerve swerve,
      boolean isInitialPoint,
      Supplier<Boolean> aimAtSpeaker,
      ChassisSpeeds startingSpeeds) {
    this.pathSupplier = path;
    this.swerve = swerve;
    this.isInitialPoint = isInitialPoint;
    this.aimAtSpeaker = aimAtSpeaker;
    this.startingSpeeds = startingSpeeds;
    addRequirements(swerve);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Swerve swerve,
      Supplier<Boolean> aimAtSpeaker,
      ChassisSpeeds startingSpeeds) {
    this(path, swerve, false, aimAtSpeaker, startingSpeeds);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Swerve swerve,
      boolean isInitialPoint,
      Supplier<Boolean> aimAtSpeaker) {
    this(path, swerve, isInitialPoint, aimAtSpeaker, new ChassisSpeeds());
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path, Swerve swerve, Supplier<Boolean> aimAtSpeaker) {
    this(path, swerve, false, aimAtSpeaker, new ChassisSpeeds());
  }

  @Override
  public void initialize() {
    Logger.recordOutput("TrajectoryFollowerCommandAlliance", Robot.alliance);
    if (!firstTime) {
      path = pathSupplier.get();
      firstTime = true;
    }
    if (Robot.alliance == Alliance.Red && !flippedForRed) {
      path = path.flipPath();
      flippedForRed = true;
    }

    if (Robot.alliance == Alliance.Blue && flippedForRed) {
      path = path.flipPath();
      flippedForRed = false;
    }
    if (isInitialPoint) {
      trajectory =
          path.getTrajectory(
              new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());

      Logger.recordOutput(
          "TrajectoryFollower/InitialHolonomicPose", path.getPreviewStartingHolonomicPose());
      swerve.resetPose(path.getPreviewStartingHolonomicPose());
    } else {
      trajectory =
          path.getTrajectory(startingSpeeds, RobotContainer.swerve.getAutoPose().getRotation());
    }
    timer.reset();
    timer.start();
    System.out.println("\n\n\n\nTrajectory time seconds:" + trajectory.getTotalTimeSeconds());
  }

  @Override
  public void execute() {
    PathPlannerTrajectory.State goal = trajectory.sample(timer.get());
    goal.targetHolonomicRotation =
        aimAtSpeaker.get()
            ? RobotContainer.visionSupplier.robotToSpeakerAngleAuto()
            : goal.targetHolonomicRotation;
    ChassisSpeeds adjustedSpeeds =
        autonomusController.calculate(RobotContainer.swerve.getAutoPose(), goal);
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
