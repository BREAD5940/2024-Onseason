package frc.robot.vision.photonvision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAprilTagVision extends SubsystemBase {
  private PhotonCamera[] cameras;
  private PhotonPoseEstimator[] photonEstimators;
  private static final double fieldBorderMargin = 0.5;
  private double xyStdDevCoefficient = 0.006;
  private double thetaStdDevCoefficient = 0.002;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private List<Pose2d> allRobotPoses;
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private Pose2d currentPose;

  private final Pose3d[] cameraPoses =
      new Pose3d[] {
        // Front Left
        new Pose3d(
            Units.inchesToMeters(6.67),
            Units.inchesToMeters(12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(180.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(15.0))),
        // Front Right
        new Pose3d(
            Units.inchesToMeters(6.67),
            Units.inchesToMeters(-12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(180.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-15.0))),
        // Back Left
        new Pose3d(
            Units.inchesToMeters(1.33),
            Units.inchesToMeters(12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(180.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(165.0))),
        // Back Right
        new Pose3d(
            Units.inchesToMeters(1.33),
            Units.inchesToMeters(-12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(180.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-165.0))),
      };

  public PhotonAprilTagVision(PhotonCamera... cameras) {
    this.cameras = cameras;
    photonEstimators = new PhotonPoseEstimator[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      photonEstimators[i] =
          new PhotonPoseEstimator(
              FieldConstants.aprilTags,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              cameras[i],
              GeomUtil.pose3dToTransform3d(cameraPoses[i]));
      photonEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Cameras Length", cameras.length);

    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      allRobotPoses = new ArrayList<>();
      visionUpdates = new ArrayList<>();
      currentPose = poseSupplier.get();

      List<Pose3d> tagPose3ds = new ArrayList<>();
      Pose2d robotPose = null;
      Pose3d cameraPose = null;
      double timestamp = 0.0;
      var latestCameraResult = cameras[instanceIndex].getLatestResult();

      Logger.recordOutput(
          "Camera " + instanceIndex + " Has Targets", latestCameraResult.hasTargets());

      if (latestCameraResult.hasTargets()) {
        photonEstimators[instanceIndex].setReferencePose(currentPose);
        var latestVisionEstimation = photonEstimators[instanceIndex].update();

        // robotPose = latestVisionEstimation.get().estimatedPose.toPose2d();
        // timestamp = latestVisionEstimation.get().timestampSeconds;
        robotPose = new Pose2d();
        timestamp = 0.0;

        System.out.println(photonEstimators[instanceIndex].getRobotToCameraTransform());

        if (latestCameraResult.getMultiTagResult().estimatedPose.isPresent) {
          cameraPose =
              GeomUtil.transform3dToPose3d(
                  latestCameraResult.getMultiTagResult().estimatedPose.best);
        } else {
          cameraPose =
              GeomUtil.transform3dToPose3d(
                  cameras[instanceIndex].getLatestResult().getBestTarget().getBestCameraToTarget());
        }

        for (PhotonTrackedTarget trackedTarget : latestVisionEstimation.get().targetsUsed) {
          int fiducialId = trackedTarget.getFiducialId();
          Pose3d tagPose = FieldConstants.aprilTags.getTagPose(fiducialId).get();

          if (tagPose != null) {
            tagPose3ds.add(tagPose);
          }
        }
      }

      // Exit if no data
      if (robotPose == null || cameraPose == null) {
        continue;
      }

      // Exit if robot pose is off the field
      if (robotPose.getX() < -fieldBorderMargin
          || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
          || robotPose.getY() < -fieldBorderMargin
          || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
        continue;
      }

      // Calculate average distance to tag
      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPose3ds) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }
      double avgDistance = totalDistance / tagPose3ds.size();
      double xyStdDev = 0.0;
      double thetaStdDev = 0.0;

      xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();

      visionUpdates.add(
          new TimestampedVisionUpdate(
              robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      allRobotPoses.add(robotPose);

      Logger.recordOutput("PhotonTagVision/VisionUpdates", visionUpdates.size());
      Logger.recordOutput("PhotonTagVision/VisionData" + instanceIndex, robotPose);
    }
  }
}
