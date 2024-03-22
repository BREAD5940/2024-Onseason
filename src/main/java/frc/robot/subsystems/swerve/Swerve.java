package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.Swerve.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.subsystems.swerve.BreadSwerveModule.DriveRequestType;
import frc.robot.subsystems.swerve.BreadSwerveRequest.FieldCentric;
import frc.robot.subsystems.swerve.BreadSwerveRequest.RobotCentric;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  /*IStores the swerve drivetrain object */
  private final BreadSwerveDrivetrain drivetrain;

  /* Stores requests and parameters */
  private ChassisSpeeds desired = new ChassisSpeeds();
  private boolean fieldRelative = false;

  private boolean requestVelocity = false;
  private boolean requestPercent = false;
  private boolean isBrakeMode = true;
  private Timer lastMovementTimer = new Timer();

  private SwerveState systemState = SwerveState.PERCENT;

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    this.drivetrain =
        new BreadSwerveDrivetrain(
            drivetrainConstants,
            251,
            VecBuilder.fill(0.01, 0.01, 0.002),
            VecBuilder.fill(0.01, 0.01, 0.002),
            moduleConstants);

    drivetrain.configNeutralMode(NeutralModeValue.Brake);

    lastMovementTimer.start();
  }

  @Override
  public void periodic() {
    handleTelemetry();
    handleStatemachineLogic();
  }

  /* Telemetry function */
  private void handleTelemetry() {
    Pose2d pose = getPose();
    SwerveModuleState[] targets = drivetrain.getState().ModuleTargets;
    SwerveModuleState[] states = drivetrain.getState().ModuleStates;
    Logger.recordOutput("Odometry/PoseEstimatorEstimate", pose);
    Logger.recordOutput("Odometry/PoseEstimatorEstimatorAuto", getPoseAuto());
    Logger.recordOutput("Swerve/Targets", targets);
    Logger.recordOutput("Swerve/Achieved", states);
    drivetrain.logCurrents();
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

    /* If the driver station is enabled, set the modules to break. Otherwise set them to coast */
    boolean stillMoving = false;
    for (int i = 0; i < 4; i++) {
      if (drivetrain.getModule(i).getCurrentState().speedMetersPerSecond
          > SWERVE_COAST_TRESHOLD_MPS) {
        stillMoving = true;
      }
    }
    if (stillMoving) lastMovementTimer.reset();
    if (DriverStation.isEnabled()) {
      if (!isBrakeMode) {
        isBrakeMode = true;
        drivetrain.configNeutralMode(NeutralModeValue.Brake);
      }
    } else {
      if (isBrakeMode && lastMovementTimer.hasElapsed(SWERVE_COAST_TRESHOLD_SEC)) {
        isBrakeMode = false;
        drivetrain.configNeutralMode(NeutralModeValue.Coast);
      }
    }
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

  /* Adds vision data to the pose estimator built into the drivetrain class */
  public void addVisionData(
      List<TimestampedVisionUpdate> visionUpdates,
      List<TimestampedVisionUpdate> visionUpdatesAuto) {
    for (TimestampedVisionUpdate update : visionUpdates) {
      drivetrain.addVisionMeasurement(update.pose(), update.timestamp(), update.stdDevs());
    }
    for (TimestampedVisionUpdate update : visionUpdatesAuto) {
      drivetrain.addVisionMeasurementAuto(update.pose(), update.timestamp(), update.stdDevs());
    }
  }

  /* Resets the pose estimate of the robot */
  public void resetPose(Pose2d newPose) {
    drivetrain.seedFieldRelative(newPose);
  }

  /* Returns the current pose estimate of the robot */
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  /* Returns the current pose estimator of the robot (auto for path following) */
  public Pose2d getPoseAuto() {
    return drivetrain.getState().PoseAuto;
  }

  /* Returns the robot relative speeds of the robot */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drivetrain.getState().speeds;
  }

  /* Returns the field relative speeds of the robot */
  public Translation2d getFieldRelativeSpeeds() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();

    Translation2d speeds2d = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    Translation2d fieldRelativeSpeeds2d = speeds2d.rotateBy(getPose().getRotation());

    return fieldRelativeSpeeds2d;
  }

  public boolean atAngularSetpoint(double setpointRad) {
    return Math.abs(this.getPose().getRotation().getRadians() - setpointRad)
        < SWERVE_ANGULAR_ERROR_TOLERANCE_RAD;
  }

  /* Swerve State */
  public enum SwerveState {
    VELOCITY,
    PERCENT
  }
}
