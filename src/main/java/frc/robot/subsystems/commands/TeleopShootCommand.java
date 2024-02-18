package frc.robot.subsystems.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.subsystems.swerve.Swerve;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TeleopShootCommand extends Command {

  // Subsystems
  private Swerve swerve;
  private boolean converged = false;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0);

  public TeleopShootCommand(Swerve swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    converged = false;
  }

  @Override
  public void execute() {
    double setpoint = RobotContainer.visionSupplier.robotToSpeakerAngle().getRadians();
    double measurement = swerve.getPose().getRotation().getRadians();

    double output = turnPID.calculate(measurement, setpoint);

    if (RobotContainer.aprilTagVision.seesTag4()) {
      // Add vision data
      Pose2d estimatedRobotPose = RobotContainer.aprilTagVision.getRobotToSpeakerTag();
      TimestampedVisionUpdate update =
          new TimestampedVisionUpdate(
              estimatedRobotPose,
              RobotContainer.aprilTagVision.getSpeakerTagTimestamp(),
              VecBuilder.fill(0.00000005, 0.0000005, 1.0E6));
      RobotContainer.swerve.addVisionData(List.of(update));

      // Calculate whether or not the pose estimate has converged to the data we're recieving from
      // the speaker tag
      Pose2d convergenceError = estimatedRobotPose.relativeTo(RobotContainer.swerve.getPose());
      if (convergenceError.getX() < Units.inchesToMeters(1.0)
          && convergenceError.getY() < Units.inchesToMeters(1.0)
          && convergenceError.getRotation().getDegrees() < 1.0) {
        converged = true;
      }
    }

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(0, 0, output), true);
    RobotContainer.superstructure.requestVisionSpeaker(
        true, converged && RobotContainer.swerve.atAngularSetpoint(setpoint), false);
    RobotContainer.shooter.requestVisionSpeaker(false);

    // Logs
    Logger.recordOutput(
        "AimAtSpeakerCommand/measurementDegrees", Units.radiansToDegrees(measurement));
    Logger.recordOutput("AimAtSpeakerCommand/setpointDegrees", Units.radiansToDegrees(setpoint));
    Logger.recordOutput("TeleopShootCommand/converged", converged);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.requestIdle();
    RobotContainer.superstructure.requestVisionSpeaker(false, false, false);
  }
}
