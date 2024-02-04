package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NotePoseSupplier extends SubsystemBase {
  private static PhotonCamera camera = new PhotonCamera("ObjectDetectionCamera");

  private Transform2d robotToCamera =
      new Transform2d(
          new Translation2d(Units.inchesToMeters(-20), 0.0),
          new Rotation2d(0.0, 0)); // flip translations
  private Transform2d cameraToTarget;
  private double yaw;
  private double pitch;
  private double area;
  private double skew;
  private Pose2d latestNotePose = new Pose2d();

  public void periodic() {
    // Get robot pose
    Pose2d robotPose = RobotContainer.swerve.getPose();

    // Get camera to note transf
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();

      pitch = target.getPitch() + 7.74;
      area = target.getArea();
      skew = target.getSkew();
      yaw = -target.getYaw() - 8.32;

      Translation2d cameraToTargetTranslation = getCameraToTarget();
      cameraToTarget = new Transform2d(cameraToTargetTranslation, new Rotation2d());

      // Apply the camera to target transformation first, then apply the camera to robot
      // transformation.
      Transform2d robotToTarget = robotToCamera.plus(cameraToTarget);

      // Transform robot pose by the robot to target transformation.
      // This provides the target's global position on the field.
      latestNotePose = robotPose.transformBy(robotToTarget);
    }
  }

  // Method to get the area
  public double getArea() {
    return area;
  }
  // Method to get the pitch
  public double getPitch() {
    return pitch;
  }

  // Method to get the pitch
  public double getYaw() {
    return yaw;
  }

  // Method to get the skew
  public double getSkew() {
    return skew;
  }

  public Pose2d getLatestNotePose() {
    return latestNotePose;
  }

  // Private method that returns the distance to the target
  private Translation2d getCameraToTarget() {
    // Define the vector representing direction from the camera to the target, using yaw and pitch
    // Conver to yaw and pitch into cartesian coordinates (x, y)
    double x = 1.0 * Math.tan(Units.degreesToRadians(yaw));
    double y = 1.0 * Math.tan(Units.degreesToRadians(pitch));
    double z = 1.0;

    // v = (x, y, z) is: ||v|| = sqrt(x^2 + y^2 + z^2).
    double norm = Math.sqrt(x * x + y * y + z * z);

    // Normalize vector by dividing by its magnitude (accounts for the spherical issue that couples
    // x and y angles.)
    x /= norm;
    y /= norm;
    z /= norm;

    // Rotate the (y, z) vector by the camera pitch
    Translation2d yzPrime =
        new Translation2d(y, z)
            .rotateBy(
                new Rotation2d(-Units.degreesToRadians(-20.0))); // updated the mounting pitch sign

    double yPrime = yzPrime.getX(); // get yPrime from the rotated vector

    // Solve for the intersection between the 3d vector and the known height of the target
    // Calculate the angle to the goal based on the y of the rotated vector
    double angleToGoalRadians = Math.asin(yPrime);

    // Get the absolute height difference between the target and camera lens
    double diffHeight = Math.abs(1.0 - Units.inchesToMeters(19.0));

    // Calculate the distance to the goal
    // The tan of the angle is used to relate the vertical displacement to the distance
    double distance = (diffHeight / Math.tan(angleToGoalRadians));

    // Returns the distance to the target (in meters), with the yaw
    return new Translation2d(distance, Rotation2d.fromDegrees(yaw - Units.radiansToDegrees(0.0)));
  }
}
