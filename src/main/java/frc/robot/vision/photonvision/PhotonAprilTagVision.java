package frc.robot.vision.photonvision;

import static frc.robot.constants.FieldConstants.aprilTags;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.PolynomialRegression;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAprilTagVision extends SubsystemBase {
  private PhotonCamera[] cameras;
  private static final double fieldBorderMargin = 0.5;
  private double xyStdDevCoefficient = 0.006;
  private double thetaStdDevCoefficient = 0.002;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private double stdDevScalar = 1.0;

  private double speakerTagTimestamp = 0.0;
  private Pose3d lastRobotToSpeakerTagPose = new Pose3d();
  private boolean seesTag4 = false;

  public enum StdDevMode {
    DEFAULT,
    SHOOTING
  }

  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);
  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

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
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {

    Pose2d currentPose = poseSupplier.get();
    visionUpdates = new ArrayList<>();

    // Loop through all the cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {

      // Camera-specific variables
      Pose3d cameraPose;
      Pose2d robotPose;
      List<Pose3d> tagPose3ds = new ArrayList<>();
      PhotonPipelineResult latestCameraResult = cameras[instanceIndex].getLatestResult();

      Logger.recordOutput(
          "Photon/Camera " + instanceIndex + " Has Targets", latestCameraResult.hasTargets());

      // Continue if the camera doesn't have any targets
      if (!latestCameraResult.hasTargets()) {
        continue;
      }

      double timestamp = latestCameraResult.getTimestampSeconds();
      Logger.recordOutput("Photon/Camera " + instanceIndex + " Timestamp", timestamp);

      // Handle speaker distance and angle calculations
      if (instanceIndex == 0) {
        seesTag4 = handleSpeakerVisionCalcs(latestCameraResult.targets, timestamp);
      }

      boolean shouldUseMultiTag = latestCameraResult.getMultiTagResult().estimatedPose.isPresent;

      if (shouldUseMultiTag) {
        // If multitag, use directly
        cameraPose =
            GeomUtil.transform3dToPose3d(latestCameraResult.getMultiTagResult().estimatedPose.best);

        robotPose =
            cameraPose
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();

        // Populate array of tag poses with tags used
        for (int id : latestCameraResult.getMultiTagResult().fiducialIDsUsed) {
          tagPose3ds.add(aprilTags.getTagPose(id).get());
        }

        Logger.recordOutput("Photon/Camera Pose (Multi tag) " + instanceIndex, cameraPose);
      } else {
        // If not using multitag, disambiugate and then use
        PhotonTrackedTarget target = latestCameraResult.targets.get(0);

        Pose3d tagPos = aprilTags.getTagPose(target.getFiducialId()).get();

        Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
        Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());
        Pose2d robotPose0 =
            cameraPose0
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();
        Pose2d robotPose1 =
            cameraPose1
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();

        double projectionError = target.getPoseAmbiguity();

        // Select a pose using projection error and current rotation
        if (projectionError < 0.15) {
          cameraPose = cameraPose0;
          robotPose = robotPose0;
        } else if (Math.abs(robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
            < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
          cameraPose = cameraPose0;
          robotPose = robotPose0;
        } else {
          cameraPose = cameraPose1;
          robotPose = robotPose1;
        }

        tagPose3ds.add(tagPos);
        Logger.recordOutput("Photon/Camera Pose (Single Tag) " + instanceIndex, cameraPose);
      }

      // TODO not sure if we need this but I'll just leave it here
      if (cameraPose == null || robotPose == null) {
        continue;
      }

      // Move on to next camera if robot pose is off the field
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

      if (shouldUseMultiTag) {
        xyStdDev =
            stdDevScalar * xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();
        thetaStdDev =
            stdDevScalar * thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      } else {
        xyStdDev = xyStdDevModel.predict(avgDistance) * stdDevScalar;
        thetaStdDev = thetaStdDevModel.predict(avgDistance) * stdDevScalar;
      }

      visionUpdates.add(
          new TimestampedVisionUpdate(
              robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));

      Logger.recordOutput("VisionData/" + instanceIndex, robotPose);
      Logger.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());
      Logger.recordOutput("Photon/LastRobotToSpeakerTag", lastRobotToSpeakerTagPose);
      Logger.recordOutput("Photon/SpeakerTagTimestamp", getSpeakerTagTimestamp());
      Logger.recordOutput(
          "Photon/RobotToSpeakerTagDistance",
          aprilTags
              .getTagPose(4)
              .get()
              .toPose2d()
              .relativeTo(lastRobotToSpeakerTagPose.toPose2d()));
    }

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);
  }

  public void setStdDevMode(StdDevMode mode) {
    if (mode == StdDevMode.DEFAULT) {
      stdDevScalar = 1.0;
    } else if (mode == StdDevMode.SHOOTING) {
      stdDevScalar = 1.0;
    } else {
      stdDevScalar = 1.0;
    }
  }

  private boolean handleSpeakerVisionCalcs(List<PhotonTrackedTarget> targets, double timestamp) {
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == 4) { // TODO hard-coded for now but change later if works
        lastRobotToSpeakerTagPose =
            getRobotToSpeakerTag(
                aprilTags.getTagPose(4).get().getTranslation(),
                new Pose3d(poseSupplier.get()),
                target);
        speakerTagTimestamp = timestamp;
        return true;
      }
    }
    return false;
  }

  private double getXYDistanceToTarget(
      PhotonTrackedTarget detection, double targetHeightOffGround) {

    Transform3d robotToCamera =
        new Transform3d(
            cameraPoses[0].getTranslation(),
            new Rotation3d(
                cameraPoses[0].getRotation().getX(),
                cameraPoses[0].getRotation().getY(),
                cameraPoses[0].getRotation().getZ()));

    // Define the vector
    double x = 1.0 * Math.tan(Units.degreesToRadians(-detection.getYaw()));
    double y = 1.0 * Math.tan(Units.degreesToRadians(-detection.getPitch()));
    double z = 1.0;
    double norm = Math.sqrt(x * x + y * y + z * z);
    x /= norm;
    y /= norm;
    z /= norm;

    // Rotate the vector by the camera pitch
    double xPrime = x;
    Translation2d yzPrime =
        new Translation2d(y, z).rotateBy(new Rotation2d(robotToCamera.getRotation().getY()));
    double yPrime = yzPrime.getX();
    double zPrime = yzPrime.getY();

    // Solve for the intersection
    double angleToTargetRadians = Math.asin(yPrime);
    double diffHeight = targetHeightOffGround - robotToCamera.getZ();
    double distance = diffHeight / Math.tan(angleToTargetRadians);

    Logger.recordOutput("Photon/SpeakerDistance", distance);
    return distance;
  }

  private Pose3d getRobotToSpeakerTag(
      Translation3d target, Pose3d robotPose, PhotonTrackedTarget detection) {
    Transform3d robotToCamera =
        new Transform3d(
            cameraPoses[0].getTranslation(),
            new Rotation3d(
                cameraPoses[0].getRotation().getX(),
                cameraPoses[0].getRotation().getY(),
                cameraPoses[0].getRotation().getZ()));

    double targetHeightOffGround = target.getZ();
    double xyDistanceToTarget = getXYDistanceToTarget(detection, targetHeightOffGround);
    double distanceToTarget =
        Math.hypot(xyDistanceToTarget, targetHeightOffGround - robotToCamera.getZ());

    Pose3d estimatedCameraPose = robotPose.transformBy(robotToCamera);

    Logger.recordOutput("Photon/EstimatedCameraPose (Speaker Tag)", estimatedCameraPose);

    Translation3d cameraToTargetTranslation =
        new Translation3d(
            distanceToTarget,
            new Rotation3d(
                0.0,
                Units.degreesToRadians(-detection.getPitch()),
                Units.degreesToRadians(-detection.getYaw())));
    cameraToTargetTranslation =
        cameraToTargetTranslation.rotateBy(estimatedCameraPose.getRotation());

    Translation3d cameraTranslation = target.minus(cameraToTargetTranslation);
    Pose3d cameraPose = new Pose3d(cameraTranslation, estimatedCameraPose.getRotation());

    Pose3d estimatedRobotPose = cameraPose.transformBy(robotToCamera.inverse());

    return estimatedRobotPose;
  }

  public Pose2d getRobotToSpeakerTag() {
    return lastRobotToSpeakerTagPose.toPose2d();
  }

  public double getSpeakerTagTimestamp() {
    return speakerTagTimestamp;
  }

  public boolean seesTag4() {
    return seesTag4;
  }
}
