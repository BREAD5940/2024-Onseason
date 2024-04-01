package frc.robot.subsystems.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class AlignWithChainCommand extends Command {

  // Subsystems
  private Swerve swerve;

  // TODO: find this
  private final Pose2d[] hangingLocations = {
    new Pose2d(new Translation2d(4.18, 2.97), Rotation2d.fromDegrees(-120)),
    new Pose2d(new Translation2d(4.18, 5.23), Rotation2d.fromDegrees(120)),
    new Pose2d(new Translation2d(6.25, 4.07), Rotation2d.fromDegrees(0))
  };

  // Feedback controllers
  private PIDController xController = new PIDController(8, 0, 0);
  private PIDController yController = new PIDController(8, 0, 0);
  private PIDController thetaController = new PIDController(5, 0, 0);

  public AlignWithChainCommand(Swerve swerve) {
    this.swerve = swerve;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    Pose2d setpoint = AllianceFlipUtil.apply(findClosestPose());
    Pose2d measurement = swerve.getPose();

    double xOutput = xController.calculate(measurement.getX(), setpoint.getX());
    double yOutput = yController.calculate(measurement.getY(), setpoint.getY());
    double thetaOutput =
        thetaController.calculate(
            measurement.getRotation().getRadians(), setpoint.getRotation().getRadians());

    Logger.recordOutput("AlignWithChain/Setpoint", setpoint);

    for (int i = 0; i < hangingLocations.length; i++) {
      Logger.recordOutput("AlignWithChain/HangingLocation" + i, hangingLocations[i]);
    }
    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(xOutput, yOutput, thetaOutput), true);
  }

  private Pose2d findClosestPose() {
    double minDistance = Double.POSITIVE_INFINITY;
    Pose2d closestPose = null;
    Pose2d currentPose = swerve.getPose();

    // List<Pose2d> flippedHangingLocations = new ArrayList<>();

    // for (Pose2d hangingLocation : hangingLocations) {
    //   Pose2d flippedPose = AllianceFlipUtil.apply(hangingLocation);
    //   flippedHangingLocations.add(flippedPose);
    // }

    for (Pose2d pose : hangingLocations) {
      double distance = currentPose.relativeTo(pose).getTranslation().getNorm();
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }

    return closestPose;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), false);
  }
}
