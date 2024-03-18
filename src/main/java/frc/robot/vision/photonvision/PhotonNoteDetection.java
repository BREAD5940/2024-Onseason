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
                Units.degreesToRadians(-165.0)))
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

  public PhotonNoteDetection(PhotonCamera... cameras) {
    this.cameras = cameras;
  }

  @Override
  public void periodic() {
    List<Translation2d> allNotePoses = new ArrayList<>();

    // Loop through all the cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      Pose3d cameraPose = cameraPoses[instanceIndex];
      Pose2d robotPose = RobotContainer.swerve.getPose();

      // Track the best note in the FOV
      PhotonPipelineResult latestCameraResult = cameras[instanceIndex].getLatestResult();

      if (!latestCameraResult.hasTargets()) {
        continue;
      }

      PhotonTrackedTarget bestTarget = latestCameraResult.getBestTarget();

      // Calculate the center pixel coordinate of the note by averaging the corners
      List<TargetCorner> corners = bestTarget.getMinAreaRectCorners();
      TargetCorner center = averageCorners(corners);

      // If the note is justified towards the right side of the frame us the left verticies and vice
      if (center.x < cameraResolutionX / 2.0) {
        // Note is on the left side of frame, use right verticies
        List<TargetCorner> rightVerticies = getRightVerticies(corners);
        if (rightVerticies.size() < 2) {
          continue;
        }

        // Figure out which of the corners is the top right and which is the bottom
        TargetCorner topRight;
        TargetCorner bottomRight;
        if (rightVerticies.get(0).y < rightVerticies.get(1).y) {
          topRight = rightVerticies.get(0);
          bottomRight = rightVerticies.get(1);
        } else {
          topRight = rightVerticies.get(1);
          bottomRight = rightVerticies.get(0);
        }

        double[] pitchYawTopRight = getPitchYawFromTargetCorner(topRight, instanceIndex);
        double[] pitchYawBottomRight = getPitchYawFromTargetCorner(bottomRight, instanceIndex);

        Translation2d translationTopRight =
            getCameraToTarget(
                pitchYawTopRight[0], pitchYawTopRight[1], instanceIndex, noteThickness);
        Translation2d translationBottomRight =
            getCameraToTarget(pitchYawBottomRight[0], pitchYawBottomRight[1], instanceIndex, 0.0);

        Pose2d topRightVertexPose =
            robotPose
                .plus(GeomUtil.poseToTransform(cameraPoses[0].toPose2d()))
                .plus(GeomUtil.translationToTransform(translationTopRight));

        Pose2d bottomRightVertexPose =
            robotPose
                .plus(GeomUtil.poseToTransform(cameraPoses[0].toPose2d()))
                .plus(GeomUtil.translationToTransform(translationBottomRight));

        Translation2d averageVertexPose =
            averageTranslations(
                topRightVertexPose.getTranslation(), bottomRightVertexPose.getTranslation());

        Translation2d topRightToBottomRight =
            topRightVertexPose.getTranslation().minus(bottomRightVertexPose.getTranslation());

        Rotation2d angleOffset =
            new Rotation2d(topRightToBottomRight.getX(), topRightToBottomRight.getY())
                .rotateBy(Rotation2d.fromDegrees(90.0));

        Translation2d estimatedNotePose =
            averageVertexPose.plus(new Translation2d(noteWidth / 2.0, angleOffset));
        allNotePoses.add(estimatedNotePose);

        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Top Pose", topRightVertexPose);
        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Bottom Pose",
            bottomRightVertexPose);
        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Average Vertex Pose",
            new Pose2d(averageVertexPose, new Rotation2d()));
        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Note Pose",
            new Pose2d(notePose, new Rotation2d()));

      } else {
        // Note is on the right side of frame, use left verticies
        List<TargetCorner> leftVerticies = getLeftVerticies(corners);
        if (leftVerticies.size() < 2) {
          continue;
        }

        // Figure out which of the corners is the top left and which is the bottom
        TargetCorner topLeft;
        TargetCorner bottomLeft;
        if (leftVerticies.get(0).y < leftVerticies.get(1).y) {
          topLeft = leftVerticies.get(0);
          bottomLeft = leftVerticies.get(1);
        } else {
          topLeft = leftVerticies.get(1);
          bottomLeft = leftVerticies.get(0);
        }

        double[] pitchYawTopLeft = getPitchYawFromTargetCorner(topLeft, instanceIndex);
        double[] pitchYawBottomLeft = getPitchYawFromTargetCorner(bottomLeft, instanceIndex);

        Translation2d translationTopLeft =
            getCameraToTarget(pitchYawTopLeft[0], pitchYawTopLeft[1], instanceIndex, noteThickness);
        Translation2d translationBottomLeft =
            getCameraToTarget(pitchYawBottomLeft[0], pitchYawBottomLeft[1], instanceIndex, 0.0);

        Pose2d topLeftVertexPose =
            robotPose
                .plus(GeomUtil.poseToTransform(cameraPoses[0].toPose2d()))
                .plus(GeomUtil.translationToTransform(translationTopLeft));

        Pose2d bottomLeftVertexPose =
            robotPose
                .plus(GeomUtil.poseToTransform(cameraPoses[0].toPose2d()))
                .plus(GeomUtil.translationToTransform(translationBottomLeft));

        Translation2d averageVertexPose =
            averageTranslations(
                topLeftVertexPose.getTranslation(), bottomLeftVertexPose.getTranslation());

        Translation2d topLeftToBottomLeft =
            topLeftVertexPose.getTranslation().minus(bottomLeftVertexPose.getTranslation());

        Rotation2d angleOffset =
            new Rotation2d(topLeftToBottomLeft.getX(), topLeftToBottomLeft.getY())
                .rotateBy(Rotation2d.fromDegrees(-90.0));

        Translation2d estimatedNotePose =
            averageVertexPose.plus(new Translation2d(noteWidth / 2.0, angleOffset));
        allNotePoses.add(estimatedNotePose);

        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Top Pose", topLeftVertexPose);
        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Bottom Pose",
            bottomLeftVertexPose);
        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Average Vertex Pose",
            new Pose2d(averageVertexPose, new Rotation2d()));
        Logger.recordOutput(
            "NoteDetection/" + cameras[instanceIndex].getName() + "/Note Pose",
            new Pose2d(notePose, new Rotation2d()));
      }
    }

    // Filter poses and choose the best one
    List<Translation2d> filteredPoses = filterAndSelectPoses(allNotePoses);

    if (!filteredPoses.isEmpty()) {
      // Choose the best note pose from the filtered poses
      notePose = findClosestPose(filteredPoses);
      Logger.recordOutput("NoteDetection/Best Note Pose", new Pose2d(notePose, new Rotation2d()));
    }
  }

  /* Note Detection Helper Methods */
  private Translation2d findClosestPose(List<Translation2d> poses) {
    Translation2d closestPose = poses.get(0);
    double closestDistance = Double.MAX_VALUE;
    for (Translation2d pose : poses) {
      double distance = pose.getDistance(RobotContainer.swerve.getPose().getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
      }
    }
    return closestPose;
  }

  private List<Translation2d> filterAndSelectPoses(List<Translation2d> allPoses) {
    List<Translation2d> mergedPoses = mergePoses(allPoses);

    // Sort poses based on their distance from the robot
    mergedPoses.sort(
        (pose1, pose2) -> {
          double distance1 = pose1.getDistance(RobotContainer.swerve.getPose().getTranslation());
          double distance2 = pose2.getDistance(RobotContainer.swerve.getPose().getTranslation());
          return Double.compare(distance1, distance2);
        });

    // Select the top four poses
    return mergedPoses.subList(0, Math.min(4, mergedPoses.size()));
  }

  private List<Translation2d> mergePoses(List<Translation2d> poses) {
    List<Translation2d> mergedPoses = new ArrayList<>(poses);
    for (int i = 0; i < mergedPoses.size(); i++) {
      for (int j = i + 1; j < mergedPoses.size(); j++) {
        Translation2d pose1 = mergedPoses.get(i);
        Translation2d pose2 = mergedPoses.get(j);

        // Compare the distance between poses
        if (pose1.getDistance(pose2) < 0.1) { // Adjust the threshold as needed
          // Merge similar poses by averaging them
          Translation2d mergedPose = pose1.plus(pose2).times(0.5);
          // Replace the merged poses with the average
          mergedPoses.set(i, mergedPose);
          mergedPoses.set(j, mergedPose);
        }
      }
    }
    return mergedPoses;
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

  public Translation2d getNotePose() {
    return notePose;
  }
}
