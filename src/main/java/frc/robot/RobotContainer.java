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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commons.BreadUtil;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.AimAtSpeakerCommand;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.ElevatorIOKrakenX60;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.elevatorpivot.PivotIOKrakenX60;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOFalcon500;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon500;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakenX60;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.VisionSupplier;
import frc.robot.vision.photonvision.PhotonAprilTagVision;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public static ShooterIO shooterIO = new ShooterIOKrakenX60();
  public static Shooter shooter = new Shooter(shooterIO);

  public static IntakeIO intakeIO = new IntakeIOFalcon500();
  public static Intake intake = new Intake(intakeIO);

  public static ElevatorIO elevatorIO = new ElevatorIOKrakenX60();
  public static PivotIO pivotIO = new PivotIOKrakenX60();
  public static FeederIO feederIO = new FeederIOFalcon500();
  public static Superstructure superstructure = new Superstructure(elevatorIO, pivotIO, feederIO);
  public static final Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
  // public static final AprilTagVisionIO frontLeft =
  //     new AprilTagVisionIONorthstar("front-left-camera");
  // public static final AprilTagVisionIO frontRight =
  //     new AprilTagVisionIONorthstar("front-right-camera");
  // public static final AprilTagVisionIO backLeft = new
  // AprilTagVisionIONorthstar("back-left-camera");
  // public static final AprilTagVisionIO backRight =
  //     new AprilTagVisionIONorthstar("back-right-camera");
  // public static final AprilTagVision northstarVision =
  //     new AprilTagVision(frontLeft, frontRight, backLeft, backRight);

  public static final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public static final PhotonCamera frontRightCamera = new PhotonCamera("front-right");

  public static final PhotonAprilTagVision aprilTagVision =
      new PhotonAprilTagVision(frontLeftCamera, frontRightCamera);
  public static final VisionSupplier visionSupplier = new VisionSupplier();
  public static AutonomousSelector autonomousSelector;

  public RobotContainer() {
    configureBindings();
    // configureNorthstar();
    configureAprilTagVision();
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
                dx = Math.pow(-x, 1) * 8.0;
                dy = Math.pow(-y, 1) * 8.0;

              } else {
                dx = Math.pow(-x, 1) * -1 * 8.0;
                dy = Math.pow(-y, 1) * -1 * 8.0;
              }
              double rot = Math.pow(-omega, 3) * 6.0;
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

    new JoystickButton(driver, XboxController.Button.kB.value)
        .whileTrue(new AimAtSpeakerCommand(swerve));
  }

  // private void configureNorthstar() {
  //   northstarVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
  // }

  private void configureAprilTagVision() {
    aprilTagVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
  }

  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void configureAutonomousSelector() {
    autonomousSelector = new AutonomousSelector(swerve, superstructure, shooter, intake);
  }
}
