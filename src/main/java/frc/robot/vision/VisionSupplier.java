package frc.robot.vision;

import static frc.robot.constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.subsystems.shooter.InterpolatingTable;
import frc.robot.subsystems.shooter.SODInterpolatingTable;
import frc.robot.subsystems.shooter.ShotParameter;
import org.littletonrobotics.junction.Logger;

public class VisionSupplier extends SubsystemBase {

  /* Constants */
  private static double TANGENTIAL_NOTE_FLIGHT_TIME = 0.2;
  private static double RADIAL_NOTE_FLIGHT_TIME = 0.001;

  /* Speaker Results */
  private Rotation2d yaw;
  private ShotParameter shot;
  private ShotParameter shotSOD;
  private ShotParameter overrideRobotPoseShot;
  private Rotation2d robotToAmpAngle;
  private double distance;
  private Pose2d overrideRobotPose;

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

  public void setOverrideRobotPose(Pose2d overrideRobotPose) {
    this.overrideRobotPose = overrideRobotPose;
  }

  public ShotParameter getShotFromOverrideRobotPose() {
    if (overrideRobotPose != null) {
      return overrideRobotPoseShot;
    } else {
      return new ShotParameter(0.0, 0.0, 0.0, 0.0);
    }
  }

  @Override
  public void periodic() {
    /* Flip amp position depending on alliance */
    Translation2d ampCenter2d = AllianceFlipUtil.apply(ampCenter);

    /* Flip the target position if you're on the red alliance */
    Pose2d targetPose = AllianceFlipUtil.apply(targetPoseBlue.toPose2d());

    /* Get the robot's position */
    Pose2d robotPose = RobotContainer.swerve.getPose();

    /* Speaker calculations */
    Translation2d fieldRelativeRobotVelocity = RobotContainer.swerve.getFieldRelativeSpeeds();

    Translation2d tangentialVirtualTarget =
        targetPose
            .getTranslation()
            .plus(
                fieldRelativeRobotVelocity
                    .times(TANGENTIAL_NOTE_FLIGHT_TIME)
                    .rotateBy(Rotation2d.fromDegrees(180.0)));

    Translation2d radialVirtualTarget =
        targetPose
            .getTranslation()
            .plus(
                fieldRelativeRobotVelocity
                    .times(RADIAL_NOTE_FLIGHT_TIME)
                    .rotateBy(Rotation2d.fromDegrees(180.0)));

    Translation2d robotToTangentialVirtualTarget =
        tangentialVirtualTarget.minus(robotPose.getTranslation());
    Translation2d robotToRadialVirtualTarget =
        radialVirtualTarget.minus(robotPose.getTranslation());

    yaw =
        new Rotation2d(
            robotToTangentialVirtualTarget.getX(), robotToTangentialVirtualTarget.getY());

    shot = InterpolatingTable.get(robotToRadialVirtualTarget.getNorm());

    shotSOD = SODInterpolatingTable.get(robotToRadialVirtualTarget.getNorm());

    if (overrideRobotPose != null) {
      Translation2d overrideRobotPoseToVirtualTarget =
          radialVirtualTarget.minus(overrideRobotPose.getTranslation());

      overrideRobotPoseShot = InterpolatingTable.get(overrideRobotPoseToVirtualTarget.getNorm());
    }

    distance = robotToRadialVirtualTarget.getNorm();

    /* Robot To Amp */
    Translation2d robotToAmp = ampCenter2d.minus(robotPose.getTranslation());

    robotToAmpAngle = new Rotation2d(robotToAmp.getX(), robotToAmp.getY());

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
