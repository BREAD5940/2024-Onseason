package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class FieldConstants {

  public static final double fieldLength = 16.451;
  public static final double fieldWidth = 8.211;
  public static final Pose2d targetPose =
      new Pose2d(16.579342, 5.547867999999999, Rotation2d.fromDegrees(180.0));

  // AprilTag constants
  public static final double aprilTagWidth = Units.inchesToMeters(8.12500);

  public static final AprilTagFieldLayout aprilTags =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      15.079471999999997,
                      0.24587199999999998,
                      1.355852,
                      new Rotation3d(
                          new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
              new AprilTag(
                  2,
                  new Pose3d(
                      16.185134,
                      0.883666,
                      1.355852,
                      new Rotation3d(
                          new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
              new AprilTag(
                  3,
                  new Pose3d(
                      16.579342 - Units.inchesToMeters(2.75),
                      4.982717999999999,
                      1.4511020000000001,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  4,
                  new Pose3d(
                      16.579342 - Units.inchesToMeters(2.75),
                      5.547867999999999,
                      1.4511020000000001,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  5,
                  new Pose3d(
                      14.700757999999999,
                      8.2042,
                      1.355852,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  6,
                  new Pose3d(
                      1.8415,
                      8.2042,
                      1.355852,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  7,
                  new Pose3d(
                      -0.038099999999999995,
                      5.547867999999999,
                      1.4511020000000001,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  8,
                  new Pose3d(
                      -0.038099999999999995,
                      4.982717999999999,
                      1.4511020000000001,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  9,
                  new Pose3d(
                      0.356108,
                      0.883666,
                      1.355852,
                      new Rotation3d(
                          new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  10,
                  new Pose3d(
                      1.4615159999999998,
                      0.24587199999999998,
                      1.355852,
                      new Rotation3d(
                          new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  11,
                  new Pose3d(
                      11.904726,
                      3.7132259999999997,
                      1.3208,
                      new Rotation3d(
                          new Quaternion(-0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  12,
                  new Pose3d(
                      11.904726,
                      4.49834,
                      1.3208,
                      new Rotation3d(
                          new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  13,
                  new Pose3d(
                      11.220196,
                      4.105148,
                      1.3208,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  14,
                  new Pose3d(
                      5.320792,
                      4.105148,
                      1.3208,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  15,
                  new Pose3d(
                      4.641342,
                      4.49834,
                      1.3208,
                      new Rotation3d(
                          new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
              new AprilTag(
                  16,
                  new Pose3d(
                      4.641342,
                      3.7132259999999997,
                      1.3208,
                      new Rotation3d(
                          new Quaternion(-0.4999999999999998, 0.0, 0.0, 0.8660254037844387))))),
          fieldLength,
          fieldWidth);

  // Target Rectangle
  public static final Pose3d tagFourPose = aprilTags.getTagPose(4).get();

  public static final Translation3d[] targetVerticies = {
    new Translation3d(
        tagFourPose.getX(),
        tagFourPose.getY() - Units.inchesToMeters(21.125),
        tagFourPose.getZ() + Units.inchesToMeters(21.125)),
    new Translation3d(
        tagFourPose.getX(),
        tagFourPose.getY() + Units.inchesToMeters(21.125),
        tagFourPose.getZ() + Units.inchesToMeters(21.125)),
    new Translation3d(
        tagFourPose.getX() - Units.inchesToMeters(20.0),
        tagFourPose.getY() - Units.inchesToMeters(21.25),
        tagFourPose.getZ() + Units.inchesToMeters(26.35)),
    new Translation3d(
        tagFourPose.getX() - Units.inchesToMeters(20.0),
        tagFourPose.getY() + Units.inchesToMeters(21.125),
        tagFourPose.getZ() + Units.inchesToMeters(26.35))
  };
}