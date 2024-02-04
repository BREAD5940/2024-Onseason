package frc.robot.subsystems.swerve;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;

public class PathFindToPointCommand extends PathfindHolonomic {

  private static HolonomicPathFollowerConfig config =
      new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0),
          4.5,
          0.4,
          new ReplanningConfig());

  public PathFindToPointCommand(Pose2d targetPose, PathConstraints constraints, Swerve swerve) {
    super(
        targetPose,
        constraints,
        0.0,
        swerve::getPose,
        swerve::getRobotRelativeSpeeds,
        (speeds) -> swerve.requestVelocity(speeds, false),
        config,
        0.0,
        swerve);
  }
}
