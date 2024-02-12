package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

public class TunerConstants {
  static class CustomSlotGains extends Slot0Configs {
    public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kS = kS;
    }
  }

  private static final CustomSlotGains steerGains = new CustomSlotGains(130, 0, 0.05, 0, 0);
  private static final CustomSlotGains driveGains =
      new CustomSlotGains(0.05, 0, 0, 0.15 * (.7 / .94), 0);
  private static final double kCoupleRatio = 0.0;
  // private static final double kDriveGearRatio = 6.545454545;
  // private static final double kSteerGearRatio = 10.28571429;
  private static final double kDriveGearRatio = 6.75;
  private static final double kSteerGearRatio = 21.4285714286;
  private static final double kWheelRadiusInches = 1.9855196;
  private static final int kPigeonId = 30;
  private static final boolean kSteerMotorReversed = true;
  private static final String kCANbusName = "dabus";
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;
  private static double kSteerInertia = 0.00001;
  private static double kDriveInertia = 0.001;
  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);
  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(800)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSpeedAt12VoltsMps(6) // Theoretical free speed is 10 meters per second at 12v applied
          // output
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(
              kCoupleRatio) // Every 1 rotation of the azimuth results in couple ratio
          // drive turns
          .withSteerMotorInverted(kSteerMotorReversed);
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 5;
  private static final int kFrontLeftEncoderId = 21;
  private static final double kFrontLeftEncoderOffset = -0.416259765625;
  private static final double kFrontLeftXPosInches = 9.375;
  private static final double kFrontLeftYPosInches = 9.375;
  private static final int kFrontRightDriveMotorId = 2;
  private static final int kFrontRightSteerMotorId = 6;
  private static final int kFrontRightEncoderId = 22;
  private static final double kFrontRightEncoderOffset = 0.46826171875;
  private static final double kFrontRightXPosInches = 9.375;
  private static final double kFrontRightYPosInches = -9.375;
  private static final int kBackLeftDriveMotorId = 3;
  private static final int kBackLeftSteerMotorId = 7;
  private static final int kBackLeftEncoderId = 23;
  // private static final double kBackLeftEncoderOffset = -0.354248046875;
  private static final double kBackLeftEncoderOffset = 0.186767578125;

  private static final double kBackLeftXPosInches = -9.375;
  private static final double kBackLeftYPosInches = 9.375;
  private static final int kBackRightDriveMotorId = 4;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 24;
  private static final double kBackRightEncoderOffset = -0.3271484375;
  private static final double kBackRightXPosInches = -9.375;
  private static final double kBackRightYPosInches = -9.375;
  public static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          Units.inchesToMeters(kFrontLeftXPosInches),
          Units.inchesToMeters(kFrontLeftYPosInches),
          kInvertLeftSide);
  public static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          Units.inchesToMeters(kFrontRightXPosInches),
          Units.inchesToMeters(kFrontRightYPosInches),
          kInvertRightSide);
  public static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          Units.inchesToMeters(kBackLeftXPosInches),
          Units.inchesToMeters(kBackLeftYPosInches),
          kInvertLeftSide);
  public static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          Units.inchesToMeters(kBackRightXPosInches),
          Units.inchesToMeters(kBackRightYPosInches),
          kInvertRightSide);
}
