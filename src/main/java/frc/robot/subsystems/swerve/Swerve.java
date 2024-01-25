package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  /*IStores the swerve drivetrain object */
  private final SwerveDrivetrain drivetrain;

  /* Stores requests and parameters */
  private ChassisSpeeds desired = new ChassisSpeeds();
  private boolean fieldRelative = false;

  private boolean requestVelocity = false;
  private boolean requestPercent = false;

  private SwerveState systemState = SwerveState.PERCENT;

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    this.drivetrain =
        new SwerveDrivetrain(
            drivetrainConstants,
            251,
            VecBuilder.fill(0.003, 0.003, 0.0002),
            VecBuilder.fill(0.003, 0.003, 0.0002),
            moduleConstants);
  }

  @Override
  public void periodic() {
    handleTelemetry();
    handleStatemachineLogic();
  }

  /* Telemetry function */
  private void handleTelemetry() {
    Pose2d pose = drivetrain.getState().Pose;
    Logger.recordOutput("Odometry/PoseEstimatorEstimate", pose);
  }

  /* Handles statemachine logic */
  private void handleStatemachineLogic() {
    SwerveState nextSystemState = systemState;
    if (systemState == SwerveState.PERCENT) {
      /* State outputs */
      if (fieldRelative) {
        drivetrain.setControl(
            new FieldCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      } else {
        drivetrain.setControl(
            new RobotCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }

      /* State transitions */
      if (requestVelocity) {
        nextSystemState = SwerveState.VELOCITY;
      }
    } else if (systemState == SwerveState.VELOCITY) {
      /* State outputs */
      if (fieldRelative) {
        drivetrain.setControl(
            new FieldCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      } else {
        drivetrain.setControl(
            new RobotCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }

      /* State transitions */
      if (requestPercent) {
        nextSystemState = SwerveState.PERCENT;
      }
    }
    systemState = nextSystemState;
  }

  /* Request the drivetrain to drive at the specified velocity
   * "speeds" should be in meters per second
   */
  public void requestVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
    requestVelocity = true;
    requestPercent = false;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Request the drivetrain to drive at the specified velocity OPEN LOOP
   * "speeds" should be in meters per second
   */
  public void requestPercent(ChassisSpeeds speeds, boolean fieldRelative) {
    requestVelocity = false;
    requestPercent = true;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Resets the pose estimate of the robot */
  public void resetPose(Pose2d newPose) {
    drivetrain.seedFieldRelative(newPose);
  }

  /* Swerve State */
  public enum SwerveState {
    VELOCITY,
    PERCENT
  }
}
