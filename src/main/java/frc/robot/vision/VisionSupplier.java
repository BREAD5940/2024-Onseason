package frc.robot.vision;

import static frc.robot.constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.shooter.InterpolatingTableDtech;
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

  /* Speaker Results */
  private Rotation2d yaw;
  private ShotParameter shot;
  private ShotParameter shotSOD;
  private Rotation2d robotToAmpAngle;
  private double distance;
  private Rotation2d robotToPassingAngle;

  private Rotation2d robotToNoteAngle;
  private Translation2d notePose;

  public Rotation2d robotToPassingAngle() {
    return robotToPassingAngle;
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

    yaw =
        new Rotation2d(
            robotToTangentialVirtualTarget.getX(), robotToTangentialVirtualTarget.getY());

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      shot = InterpolatingTableDtech.get(robotToRadialVirtualTarget.getNorm());
      shotSOD = SODInterpolatingTableBlue.get(robotToRadialVirtualTarget.getNorm());
      Logger.recordOutput("Using Blue Table", true);
    } else {
      shot = InterpolatingTableDtech.get(robotToRadialVirtualTarget.getNorm());
      shotSOD = SODInterpolatingTableRed.get(robotToRadialVirtualTarget.getNorm());
      Logger.recordOutput("Using Blue Table", false);
    }

    distance = robotToRadialVirtualTarget.getNorm();

    /* Note follower calculations */
    notePose = RobotContainer.noteDetection.getNotePose();

    Translation2d robotToNote = notePose.minus(robotPose.getTranslation());
    robotToNoteAngle =
        new Rotation2d(robotToNote.getX(), robotToNote.getY())
            .rotateBy(Rotation2d.fromDegrees(180.0));

    /* Passing target calculation */
    Translation2d flippedPassingTarget = AllianceFlipUtil.apply(passingTarget);
    Translation2d robotToPassingPose = flippedPassingTarget.minus(robotPose.getTranslation());
    robotToPassingAngle = new Rotation2d(robotToPassingPose.getX(), robotToPassingPose.getY());

    // Logs
    Logger.recordOutput("Vision/DistanceToTarget", robotToRadialVirtualTarget.getNorm());
    Logger.recordOutput("Vision/TargetPose", targetPose);
    Logger.recordOutput(
        "Vision/RadialVirtualTarget", new Pose2d(radialVirtualTarget, new Rotation2d()));
    Logger.recordOutput(
        "Vision/TangentialVirtualTarget", new Pose2d(tangentialVirtualTarget, new Rotation2d()));
    Logger.recordOutput("Vision/Target", targetPose);
  }
}
