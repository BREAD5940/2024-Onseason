package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commons.BreadUtil;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;
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
  public static final AprilTagVisionIO frontLeft = new AprilTagVisionIONorthstar("front-left-camera");
  public static final AprilTagVisionIO frontRight = new AprilTagVisionIONorthstar("front-right-camera");
  public static final AprilTagVisionIO backLeft = new AprilTagVisionIONorthstar("back-left-camera");
  public static final AprilTagVisionIO backRight = new AprilTagVisionIONorthstar("back-right-camera");
  public static final AprilTagVision northstarVision = new AprilTagVision(frontLeft, frontRight, backLeft, backRight);

  public RobotContainer() {
    configureBindings();
    configureNorthstar();
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
                dx = Math.pow(-x, 1) * 4.0;
                dy = Math.pow(-y, 1) * 4.0;

              } else {
                dx = Math.pow(-x, 1) * -1 * 4.0;
                dy = Math.pow(-y, 1) * -1 * 4.0;
              }
              double rot = Math.pow(-omega, 3) * 3.0;
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

  private void configureNorthstar() {
    northstarVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
    }
  

  public Command getAutonomousCommand() {
    return new TrajectoryFollowerCommand(Robot.sixNoteA, swerve, true)
        .andThen(new TrajectoryFollowerCommand(Robot.sixNoteB, swerve))
        .andThen(new TrajectoryFollowerCommand(Robot.sixNoteC, swerve));
  }

}
