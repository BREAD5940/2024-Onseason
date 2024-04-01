package frc.robot.vision.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.GeomUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonNoteDetection extends SubsystemBase {

  private final PhotonCamera[] cameras;

  private final Pose3d[] cameraPoses =
      new Pose3d[] {
        // Object detection left
        new Pose3d(
            Units.inchesToMeters(1.33),
            Units.inchesToMeters(12.74),
            Units.inchesToMeters(22.74),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(25.0),
                Units.degreesToRadians(-15))),

        // Object detection right
        new Pose3d(
            Units.inchesToMeters(1.33),
            Units.inchesToMeters(-12.74),
            Units.inchesToMeters(22.74),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(25.0),
                Units.degreesToRadians(15)))
      };

  // IF THE STREAM RESOLUTION IS CHANGED, CHANGE THESE AS WELL
  private final double cameraResolutionX = 800.0;
  private final double cameraResolutionY = 600.0;

  // IF THE LENSES ARE CHANGED, CHECK THESE
  private final double[] horizontalFOV =
      new double[] {Units.degreesToRadians(70.36), Units.degreesToRadians(71.07)};
  private final double[] verticalFOV =
      new double[] {Units.degreesToRadians(43.38), Units.degreesToRadians(43.61)};

  private final double noteThickness = Units.inchesToMeters(2.0);
  private final double noteWidth = Units.inchesToMeters(14.0);

  private Translation2d notePose = new Translation2d();
  private Translation2d[] notePoses = new Translation2d[5];

  public PhotonNoteDetection(PhotonCamera... cameras) {
    this.cameras = cameras;
  }

  @Override
  public void periodic() {
    List<Translation2d> allNotePoses = new ArrayList<>();
    Arrays.fill(notePoses, null);

    // Loop through all the cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      Pose3d cameraPose = cameraPoses[instanceIndex];
      Pose2d robotPose = RobotContainer.swerve.getPose();

      // Track the best note in the FOV
      PhotonPipelineResult latestCameraResult = cameras[instanceIndex].getLatestResult();

      if (!latestCameraResult.hasTargets()) {
        continue;
      }

      for (PhotonTrackedTarget target : latestCameraResult.getTargets()) {
        // Calculate the center pixel coordinate of the note by averaging the corners
        List<TargetCorner> corners = target.getMinAreaRectCorners();
        TargetCorner center = averageCorners(corners);

        // If the note is justified towards the right side of the frame us the left verticies and
        // vice versa
        boolean useRightVerticies = false;
        if (center.x < cameraResolutionX / 2.0) {
          useRightVerticies = true;
        }

        // Extract the right or left verticies from the target box
        List<TargetCorner> verticies;
        if (useRightVerticies) {
          verticies = getRightVerticies(corners);
        } else {
          verticies = getLeftVerticies(corners);
        }

        // Figure out which verticies are the top and bottom
        TargetCorner top;
        TargetCorner bottom;
        if (verticies.get(0).y < verticies.get(1).y) {
          top = verticies.get(0);
          bottom = verticies.get(1);
        } else {
          top = verticies.get(1);
          bottom = verticies.get(0);
        }

        // Get the pitch and yaw for the top and bottom verticies
        double[] pitchYawTopLeft = getPitchYawFromTargetCorner(top, instanceIndex);
        double[] pitchYawBottomLeft = getPitchYawFromTargetCorner(bottom, instanceIndex);

        // Figure out the translation from the robot to the top and bottom verticies
        Translation2d translationTop =
            getCameraToTarget(pitchYawTopLeft[0], pitchYawTopLeft[1], instanceIndex, noteThickness);
        Translation2d translationBottom =
            getCameraToTarget(pitchYawBottomLeft[0], pitchYawBottomLeft[1], instanceIndex, 0.0);

        // Estimate the field poses of the two verticies based on the robot's position and the translations 
        // above
        Pose2d topVertexPose =
            robotPose
                .plus(GeomUtil.poseToTransform(cameraPose.toPose2d()))
                .plus(GeomUtil.translationToTransform(translationTop));

        Pose2d bottomVertexPose =
            robotPose
                .plus(GeomUtil.poseToTransform(cameraPose.toPose2d()))
                .plus(GeomUtil.translationToTransform(translationBottom));

        // Average the two vertex poses to get the point in between them
        Translation2d averageVertexPose =
            averageTranslations(
                topVertexPose.getTranslation(), bottomVertexPose.getTranslation());

        Translation2d topToBottom =
            topVertexPose.getTranslation().minus(bottomVertexPose.getTranslation());

        // Get the angular offset to apply based on whether we are using the left or right verticies
        Rotation2d angleOffset;
        if (useRightVerticies) {
          angleOffset =
              new Rotation2d(topToBottom.getX(), topToBottom.getY())
                  .rotateBy(Rotation2d.fromDegrees(90.0));
        } else {
          angleOffset = new Rotation2d(topToBottom.getX(), topToBottom.getY())
              .rotateBy(Rotation2d.fromDegrees(-90.0));
        }

        // Cook
        Translation2d estimatedNotePose =
            averageVertexPose.plus(new Translation2d(noteWidth / 2.0, angleOffset));

        allNotePoses.add(estimatedNotePose);
      }
    }
    for (int i = 0; i < allNotePoses.size(); i++) {
      Logger.recordOutput("PhotonNoteDetection/All Detections/" + i, allNotePoses.get(i));
    }
  }

  // Average target corners
  private TargetCorner averageCorners(List<TargetCorner> corners) {
    double x = 0;
    double y = 0;
    double num = 0;
    for (TargetCorner corner : corners) {
      x += corner.x;
      y += corner.y;
      num += 1;
    }
    return new TargetCorner(x / num, y / num);
  }

  // Average translations
  private Translation2d averageTranslations(Translation2d one, Translation2d two) {
    return new Translation2d((one.getX() + two.getX()) / 2.0, (one.getY() + two.getY()) / 2.0);
  }

  // Converts target corner to yaw/pitch for the given camera
  private double[] getPitchYawFromTargetCorner(TargetCorner corner, int instanceIndex) {
    System.out.println(corner.x + " " + corner.y);
    // Normalize the pixel coordinates
    double nx = (1 / (cameraResolutionX / 2.0)) * (corner.x - (cameraResolutionX / 2 - 0.5));
    double ny = (1 / (cameraResolutionY / 2.0)) * ((cameraResolutionY / 2 - 0.5) - corner.y);

    // Calculate the horizontal and vertical view planes
    double vpw = 2.0 * Math.tan(horizontalFOV[instanceIndex] / 2.0);
    double vph = 2.0 * Math.tan(verticalFOV[instanceIndex] / 2.0);

    // Calculate view plane coordinates
    double x = vpw / 2.0 * nx;
    double y = vph / 2.0 * ny;

    // Calculate the yaw and pitch
    double yaw = -Units.radiansToDegrees(Math.atan2(x, 1));
    double pitch = Units.radiansToDegrees(Math.atan2(y, 1));

    return new double[] {yaw, pitch};
  }

  // Private method that returns the distance to the target
  public Translation2d getCameraToTarget(
      double yaw, double pitch, int instanceIndex, double noteHeightMeters) {
    // Define the vector
    double x = 1.0 * Math.tan(Units.degreesToRadians(yaw));
    double y = 1.0 * Math.tan(Units.degreesToRadians(pitch));
    double z = 1.0;
    double norm = Math.sqrt(x * x + y * y + z * z);
    x /= norm;
    y /= norm;
    z /= norm;

    // Rotate the vector by the camera pitch
    double xPrime = x;
    Translation2d yzPrime =
        new Translation2d(y, z)
            .rotateBy(new Rotation2d(cameraPoses[instanceIndex].getRotation().getY()));
    double yPrime = yzPrime.getX();
    double zPrime = yzPrime.getY();

    // Solve for the intersection
    double angleToGoalRadians = Math.asin(yPrime);
    double diffHeight = cameraPoses[instanceIndex].getZ() - noteHeightMeters;
    double distance = diffHeight / Math.tan(angleToGoalRadians);

    // Returns the distance to the target (in meters)
    return new Translation2d(distance, Rotation2d.fromDegrees(yaw));
  }

  // Get left verticies
  public List<TargetCorner> getLeftVerticies(List<TargetCorner> corners) {

    double minX = cameraResolutionX;
    for (TargetCorner corner : corners) {
      if (corner.x < minX) {
        minX = corner.x;
      }
    }

    List<TargetCorner> leftVerticies = new ArrayList<TargetCorner>();
    for (TargetCorner corner : corners) {
      if (Math.abs(corner.x - minX) < 0.01) {
        leftVerticies.add(corner);
      }
    }

    return leftVerticies;
  }

  // Get right verticies
  public List<TargetCorner> getRightVerticies(List<TargetCorner> corners) {

    double maxX = 0.0;
    for (TargetCorner corner : corners) {
      if (corner.x > maxX) {
        maxX = corner.x;
      }
    }

    List<TargetCorner> rightVerticies = new ArrayList<TargetCorner>();
    for (TargetCorner corner : corners) {
      if (Math.abs(corner.x - maxX) < 0.01) {
        rightVerticies.add(corner);
      }
    }

    return rightVerticies;
  }
}
