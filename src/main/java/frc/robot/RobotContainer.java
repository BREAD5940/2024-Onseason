package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commons.BreadUtil;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.northstar.AprilTagVision;
import frc.robot.vision.northstar.AprilTagVisionIO;
import frc.robot.vision.northstar.AprilTagVisionIONorthstar;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  public static AprilTagVisionIO frontLeftCamera = new AprilTagVisionIONorthstar("northstar-left");
  public static AprilTagVisionIO frontRightCamera =
      new AprilTagVisionIONorthstar("northstar-right");

  public static AprilTagVision apriltagVision =
      new AprilTagVision(frontLeftCamera, frontRightCamera);

  public RobotContainer() {
    configureBindings();
    configureNorthstarVision();
  }

  private void configureNorthstarVision() {
    apriltagVision.setDataInterfaces(swerve::getLatestPose, swerve::addVisionMeasurement);
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        new RunCommand(
            () -> {
              double x = BreadUtil.deadband(driver.getLeftY(), 0.1);
              double y = BreadUtil.deadband(driver.getLeftX(), 0.1);
              double omega = BreadUtil.deadband(driver.getRightX(), 0.1);

              double dx;
              double dy;

              if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                dx = Math.pow(-x, 1);
                dy = Math.pow(-y, 1);

              } else {
                dx = Math.pow(-x, 1) * -1;
                dy = Math.pow(-y, 1) * -1;
              }
              double rot = Math.pow(-omega, 3) * 1.5;
              swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

              if (driver.getRawButtonPressed(XboxController.Button.kStart.value)) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                  swerve.resetPose(new Pose2d());
                } else {
                  swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
                }
              }
            },
            swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
