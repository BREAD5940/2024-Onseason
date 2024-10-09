package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.Swerve.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.BreadSwerveModule.DriveRequestType;
import frc.robot.subsystems.swerve.BreadSwerveRequest.FieldCentric;
import frc.robot.subsystems.swerve.BreadSwerveRequest.PointWheelsAt;
import frc.robot.subsystems.swerve.BreadSwerveRequest.RobotCentric;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  /* Stores the swerve drivetrain object */
  private final BreadSwerveDrivetrain drivetrain;

  /* Stores requests and parameters */
  private ChassisSpeeds desired = new ChassisSpeeds();
  private boolean fieldRelative = false;

  private boolean requestVelocity = false;
  private boolean requestPercent = false;
  private boolean requestWheelRadiusCharacterization = false;
  private boolean requestOrientModules = false;
  private Rotation2d[] orientations = new Rotation2d[] {};
  private double characterizationInput = 0.0;
  private boolean isBrakeMode = true;
  private Timer lastMovementTimer = new Timer();

  private SwerveState systemState = SwerveState.PERCENT;

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    this.drivetrain =
        new BreadSwerveDrivetrain(
            drivetrainConstants,
            250,
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
    Pose2d autoPose = getAutoPose();
    SwerveModuleState[] targets = drivetrain.getState().ModuleTargets;
    SwerveModuleState[] states = drivetrain.getState().ModuleStates;
    Logger.recordOutput("Odometry/PoseEstimatorEstimate", pose);
    Logger.recordOutput("Odometry/PoseEstimatorEstimateAuto", autoPose);
    Logger.recordOutput("Swerve/Targets", targets);
    Logger.recordOutput("Swerve/Achieved", states);
    Logger.recordOutput("Swerve/OmegaRadsPerSec", getRobotRelativeSpeeds().omegaRadiansPerSecond);
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
      } else if (requestPercent) {
        nextSystemState = SwerveState.PERCENT;
      } else {
        nextSystemState = SwerveState.ORIENT_MODULES;
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
      } else if (requestWheelRadiusCharacterization) {
        nextSystemState = SwerveState.WHEEL_RADIUS_CHARACTERIZATION;
      } else if (requestOrientModules) {
        nextSystemState = SwerveState.ORIENT_MODULES;
      }
    } else if (systemState == SwerveState.WHEEL_RADIUS_CHARACTERIZATION) {
      drivetrain.setControl(
          new RobotCentric()
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(characterizationInput)
              .withDriveRequestType(DriveRequestType.Velocity));

      /* State transitions */
      if (requestPercent) {
        nextSystemState = SwerveState.PERCENT;
      } else if (requestVelocity) {
        nextSystemState = SwerveState.VELOCITY;
      } else if (requestOrientModules) {
        nextSystemState = SwerveState.ORIENT_MODULES;
      }
    } else if (systemState == SwerveState.ORIENT_MODULES) {
      Rotation2d[] circleOrientation = getCircleOrientations();

      for (int i = 0; i <= 3; i++) {
        Rotation2d currentOrientation = circleOrientation[i];

        drivetrain.setControl(new PointWheelsAt().withModuleDirection(currentOrientation));
      }

      /* State transitions */
      if (requestPercent) {
        nextSystemState = SwerveState.PERCENT;
      } else if (requestVelocity) {
        nextSystemState = SwerveState.VELOCITY;
      } else if (requestWheelRadiusCharacterization) {
        nextSystemState = SwerveState.WHEEL_RADIUS_CHARACTERIZATION;
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
    requestWheelRadiusCharacterization = false;
    requestOrientModules = false;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Request the drivetrain to drive at the specified velocity OPEN LOOP
   * "speeds" should be in meters per second
   */
  public void requestPercent(ChassisSpeeds speeds, boolean fieldRelative) {
    requestVelocity = false;
    requestPercent = true;
    requestWheelRadiusCharacterization = false;
    requestOrientModules = false;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  public void requestOrientModules(Rotation2d[] orientations) {
    requestVelocity = false;
    requestPercent = true;
    requestWheelRadiusCharacterization = false;
    requestOrientModules = true;

    this.orientations = orientations;
  }

  public void requestWheelRadiusCharacterization(double thetaSpeed) {
    requestWheelRadiusCharacterization = true;
    requestPercent = false;
    requestVelocity = false;
    requestOrientModules = false;

    this.characterizationInput = thetaSpeed;
  }

  /* Adds vision data to the pose estimator built into the drivetrain class */
  public void addVisionData(
      List<TimestampedVisionUpdate> shotVisionUpdates,
      List<TimestampedVisionUpdate> autoVisionUpdates) {
    for (TimestampedVisionUpdate shotUpdate : shotVisionUpdates) {
      drivetrain.addShotVisionMeasurement(
          shotUpdate.pose(), shotUpdate.timestamp(), shotUpdate.stdDevs());
    }
    for (TimestampedVisionUpdate autoUpdate : autoVisionUpdates) {
      drivetrain.addAutoVisionMeasurement(
          autoUpdate.pose(), autoUpdate.timestamp(), autoUpdate.stdDevs());
    }
  }

  // public double[] getWheelRadiusCharacterizationPosition() {
  //   // double[] modulePositions =
  //   //     new double[] {
  //   //       this.drivetrain.m_modulePositions[0].angle.getRadians(),
  //   //       this.drivetrain.m_modulePositions[1].angle.getRadians(),
  //   //       this.drivetrain.m_modulePositions[2].angle.getRadians(),
  //   //       this.drivetrain.m_modulePositions[3].angle.getRadians(),
  //   //     };
  //   // return modulePositions;

  //   double[] p = new double[] { 0 };
  //   return p;
  // }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(drivetrain.Modules).mapToDouble(BreadSwerveModule::getPosition).toArray();
  }

  /* Resets the pose estimate of the robot */
  public void resetPose(Pose2d newPose) {
    drivetrain.seedFieldRelative(newPose);
  }

  /* Returns the current pose estimate of the robot for path following */
  public Pose2d getPose() {
    return drivetrain.getState().ShotPose;
  }

  /* Returns the current pose estimate of the robot for shooting */
  public Pose2d getAutoPose() {
    return drivetrain.getState().AutoPose;
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

  public boolean atAngularSetpoint(double setpointRad, double tolerance) {
    return Math.abs(this.getPose().getRotation().getRadians() - setpointRad) < tolerance;
  }

  public boolean atAngularSetpoint(double setpointRad) {
    return atAngularSetpoint(setpointRad, SWERVE_ANGULAR_ERROR_TOLERANCE_RAD);
  }

  public boolean notRotating() {
    return Math.abs(this.getRobotRelativeSpeeds().omegaRadiansPerSecond)
        < SWERVE_ANGULAR_ERROR_TOLERANCE_RAD_P_S;
  }

  public Rotation2d[] getCircleOrientations() {
    return Arrays.stream(Constants.moduleTranslations)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }

  /* Swerve State */
  public enum SwerveState {
    VELOCITY,
    PERCENT,
    WHEEL_RADIUS_CHARACTERIZATION,
    ORIENT_MODULES
  }
}
