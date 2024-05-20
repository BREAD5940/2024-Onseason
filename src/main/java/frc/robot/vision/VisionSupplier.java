package frc.robot.vision;

import static frc.robot.constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.shooter.InterpolatingTableBlue;
import frc.robot.subsystems.shooter.InterpolatingTablePassing;
import frc.robot.subsystems.shooter.InterpolatingTableRed;
import frc.robot.subsystems.shooter.SODInterpolatingTableBlue;
import frc.robot.subsystems.shooter.SODInterpolatingTableRed;
import frc.robot.subsystems.shooter.ShotParameter;
import org.littletonrobotics.junction.Logger;

public class VisionSupplier extends SubsystemBase {

  /* Constants */
  private static LoggedTunableNumber TANGENTIAL_NOTE_FLIGHT_TIME =
      new LoggedTunableNumber("SOTM/TangentialNoteFlightTime", 0.4);
  private static LoggedTunableNumber RADIAL_NOTE_FLIGHT_TIME =
      new LoggedTunableNumber("SOTM/RadialNoteFlightTime", 0.3);

  private static LoggedTunableNumber PASSING_TANGENTIAL_NOTE_FLIGHT_TIME =
      new LoggedTunableNumber("SOTM/PassingTangentialNoteFlightTime", 0.5);
  private static LoggedTunableNumber PASSING_RADIAL_NOTE_FLIGHT_TIME =
      new LoggedTunableNumber("SOTM/PassingRadialNoteFlightTime", 0.5);

  /* Speaker Results */
  private Rotation2d yaw;
  private ShotParameter shot;
  private ShotParameter shotSOD;
  private Rotation2d robotToAmpAngle;
  private double distance;
  private Rotation2d robotToPassingAngle;
  private ShotParameter robotToPassingShot;
  private Translation2d[] notePoses;
  private double swerveAngleTolerance;
  private Rotation2d robotToSpeakerAngleAuto;
  private Rotation2d robotToLowPassAngle;

  private Rotation2d robotToNoteAngle;
  private Translation2d notePose;

  public int getTargetNote(int firstChoice, int secondChoice) {
    if (notePoses[firstChoice] != null) {
      return firstChoice;
    } else if (notePoses[secondChoice] != null) {
      return secondChoice;
    } else {
      return firstChoice;
    }
  }

  public Rotation2d robotToLowPassingAngle() {
    return robotToLowPassAngle;
  }

  public Rotation2d robotToSpeakerAngleAuto() {
    return robotToSpeakerAngleAuto;
  }

  public Rotation2d robotToPassingAngle() {
    return robotToPassingAngle;
  }

  public ShotParameter robotToPassingShot() {
    return robotToPassingShot;
  }

  public Rotation2d robotToNoteAngle() {
    return robotToNoteAngle;
  }

  public Translation2d notePose() {
    return notePose;
  }

  public Rotation2d robotToSpeakerAngle() {
    return yaw;
  }

  public ShotParameter robotToSpeakerShot() {
    return shot;
  }

  public ShotParameter robotToSpeakerShotSOD() {
    return shotSOD;
  }

  public Rotation2d robotToAmpAngle() {
    return robotToAmpAngle;
  }

  public double getDistance() {
    return distance;
  }

  public double getSwerveAngleTolerance() {
    return swerveAngleTolerance;
  }

  @Override
  public void periodic() {
    /* Flip the target position if you're on the red alliance */
    Pose2d targetPose = AllianceFlipUtil.apply(targetPoseBlue);

    /* Get the robot's position */
    Pose2d robotPose = RobotContainer.swerve.getPose();

    /* Speaker calculations */
    Translation2d fieldRelativeRobotVelocity = RobotContainer.swerve.getFieldRelativeSpeeds();

    Translation2d tangentialVirtualTarget =
        targetPose
            .getTranslation()
            .plus(
                fieldRelativeRobotVelocity
                    .times(TANGENTIAL_NOTE_FLIGHT_TIME.get())
                    .rotateBy(Rotation2d.fromDegrees(180.0)));

    Translation2d radialVirtualTarget =
        targetPose
            .getTranslation()
            .plus(
                fieldRelativeRobotVelocity
                    .times(RADIAL_NOTE_FLIGHT_TIME.get())
                    .rotateBy(Rotation2d.fromDegrees(180.0)));

    Translation2d robotToTangentialVirtualTarget =
        tangentialVirtualTarget.minus(robotPose.getTranslation());
    Translation2d robotToRadialVirtualTarget =
        radialVirtualTarget.minus(robotPose.getTranslation());

    Translation2d robotToTangentialVirtualTargetAuto =
        tangentialVirtualTarget.minus(RobotContainer.swerve.getAutoPose().getTranslation());

    yaw =
        new Rotation2d(
            robotToTangentialVirtualTarget.getX(), robotToTangentialVirtualTarget.getY());

    robotToSpeakerAngleAuto =
        new Rotation2d(
            robotToTangentialVirtualTargetAuto.getX(), robotToTangentialVirtualTargetAuto.getY());

    if (Robot.alliance == Alliance.Blue) {
      shot = InterpolatingTableBlue.get(robotToRadialVirtualTarget.getNorm());
      shotSOD = SODInterpolatingTableBlue.get(robotToRadialVirtualTarget.getNorm());
      Logger.recordOutput("Using Blue Table", true);
    } else {
      shot = InterpolatingTableRed.get(robotToRadialVirtualTarget.getNorm());
      shotSOD = SODInterpolatingTableRed.get(robotToRadialVirtualTarget.getNorm());
      Logger.recordOutput("Using Blue Table", false);
    }

    distance = robotToRadialVirtualTarget.getNorm();

    /* Note follower calculations */
    // notePose = RobotContainer.noteDetection.getNotePose();
    notePose = new Translation2d();

    Translation2d robotToNote = notePose.minus(robotPose.getTranslation());
    robotToNoteAngle =
        new Rotation2d(robotToNote.getX(), robotToNote.getY())
            .rotateBy(Rotation2d.fromDegrees(180.0));

    /* Passing target calculation */
    Translation2d flippedPassingTarget = AllianceFlipUtil.apply(passingTarget);

    Translation2d passingTangentialVirtualTarget =
        targetPose
            .getTranslation()
            .plus(
                fieldRelativeRobotVelocity
                    .times(PASSING_TANGENTIAL_NOTE_FLIGHT_TIME.get())
                    .rotateBy(Rotation2d.fromDegrees(180.0)));

    Translation2d passingRadialVirtualTarget =
        targetPose
            .getTranslation()
            .plus(
                fieldRelativeRobotVelocity
                    .times(PASSING_RADIAL_NOTE_FLIGHT_TIME.get())
                    .rotateBy(Rotation2d.fromDegrees(180.0)));

    Translation2d robotToPassingTangentialVirtualTarget =
        passingTangentialVirtualTarget.minus(robotPose.getTranslation());
    Translation2d robotToPassingRadialVirtualTarget =
        passingTangentialVirtualTarget.minus(robotPose.getTranslation());

    robotToPassingAngle =
        new Rotation2d(
            robotToPassingTangentialVirtualTarget.getX(),
            robotToPassingTangentialVirtualTarget.getY());
    double robotToPassingDistance = robotToPassingRadialVirtualTarget.getNorm();
    robotToPassingShot = InterpolatingTablePassing.get(robotToPassingDistance);

    /* Low Passing Target Calculations */
    Translation2d flippedLowPassingTarget = AllianceFlipUtil.apply(lowPassingTarget);

    Translation2d robotToLowPassingTarget =
        flippedLowPassingTarget.minus(robotPose.getTranslation());

    robotToLowPassAngle =
        new Rotation2d(robotToLowPassingTarget.getX(), robotToLowPassingTarget.getY());

    /* Note Poses */
    notePoses = RobotContainer.noteDetection.getNotePoses();

    /* Swerve Tolerance Calculations */
    Pose2d targetPoseA =
        new Pose2d(
            targetPose.getX(), targetPose.getY() + targetWidth / 2, targetPose.getRotation());
    Pose2d targetPoseB =
        new Pose2d(
            targetPose.getX(), targetPose.getY() - targetWidth / 2, targetPose.getRotation());

    double a = targetPoseA.getTranslation().getDistance(robotPose.getTranslation());
    double b = targetPoseB.getTranslation().getDistance(robotPose.getTranslation());
    double c = targetWidth;

    double angleToleranceRad = Math.acos((a * a + b * b - c * c) / (2 * a * b));
    swerveAngleTolerance = angleToleranceRad / 2;

    // Logs
    Logger.recordOutput("Robot To Passing Distance", robotToPassingDistance);
    Logger.recordOutput("Vision/DistanceToTarget", robotToRadialVirtualTarget.getNorm());
    Logger.recordOutput("Vision/TargetPose", targetPose);
    Logger.recordOutput(
        "Vision/RadialVirtualTarget", new Pose2d(radialVirtualTarget, new Rotation2d()));
    Logger.recordOutput(
        "Vision/TangentialVirtualTarget", new Pose2d(tangentialVirtualTarget, new Rotation2d()));
    Logger.recordOutput("Vision/Target", targetPose);
    Logger.recordOutput("Note Selection", getTargetNote(4, 3));
    Logger.recordOutput("Vision/AngleToleranceToTarget", angleToleranceRad);
    Logger.recordOutput("Vision/SwerveAngleTolerance", swerveAngleTolerance);
  }
}
