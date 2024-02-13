package frc.robot.vision;

import static frc.robot.constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.InterpolatingTable;
import frc.robot.subsystems.shooter.ShotParameter;

public class VisionSupplier extends SubsystemBase {

  /* Constants */
  private static double NOTE_FLIGHT_TIME = 0.5;

  /* Speaker Results */
  private Rotation2d yaw;
  private ShotParameter shot;

  public Rotation2d robotToSpeakerAngle() {
    return yaw;
  }

  public ShotParameter robotToSpeakerShot() {
    return shot;
  }

  @Override
  public void periodic() {
    /* Get the robot's position */
    Pose2d robotPose = RobotContainer.swerve.getPose();

    /* Speaker calculations */
    Translation2d fieldRelativeRobotVelocity = RobotContainer.swerve.getFieldRelativeSpeeds();
    Translation2d targetAdjustment =
        fieldRelativeRobotVelocity.times(NOTE_FLIGHT_TIME).rotateBy(Rotation2d.fromDegrees(180.0));
    Translation2d virtualTarget = targetPose.getTranslation().plus(targetAdjustment);

    Translation2d robotToVirtualTarget = virtualTarget.minus(robotPose.getTranslation());

    yaw = new Rotation2d(robotToVirtualTarget.getX(), robotToVirtualTarget.getY());

    shot = InterpolatingTable.get(robotToVirtualTarget.getNorm());
  }
}
