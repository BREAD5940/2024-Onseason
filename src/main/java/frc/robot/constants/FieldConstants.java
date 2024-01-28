package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.223);
  public static double fieldWidth = Units.inchesToMeters(323.277);
  public static double wingX = Units.inchesToMeters(229.201);
  public static double podiumX = Units.inchesToMeters(126.75);
  public static double startingLineX = Units.inchesToMeters(74.111);

  public static Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static double centerlineX = fieldLength / 2.0;

    public static double centerlineFirstY = Units.inchesToMeters(29.638);
    public static double centerlineSeparationY = Units.inchesToMeters(66);
    public static double spikeX = Units.inchesToMeters(114);
    public static double spikeFirstY = Units.inchesToMeters(161.638);
    public static double spikeSeparationY = Units.inchesToMeters(57);

    public static Translation2d[] centerlineTranslations = new Translation2d[5];
    public static Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    /** Center of the speaker opening (blue alliance) */
    public static Pose2d centerSpeakerOpening =
        new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), new Rotation2d());
  }

  // corners (blue alliance origin)
  public static Translation3d topRightSpeaker =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(238.815),
          Units.inchesToMeters(13.091));

  public static Translation3d topLeftSpeaker =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(197.765),
          Units.inchesToMeters(83.091));

  public static Translation3d bottomRightSpeaker =
      new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
  public static Translation3d bottomLeftSpeaker =
      new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

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
}
