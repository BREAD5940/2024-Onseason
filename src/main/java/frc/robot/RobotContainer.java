package frc.robot;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commons.BreadUtil;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.swerve.NotePoseSupplier;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static NotePoseSupplier notePoseSupplier = new NotePoseSupplier();
  public static Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  public RobotContainer() {
    configureBindings();
  }

  // Since we are using a holonomic drivetrain, the rotation component of this pose
  // represents the goal holonomic rotation
  static Pose2d targetPose = new Pose2d(11.7, 4.1, Rotation2d.fromDegrees(180));

  // Create the constraints to use while pathfinding
  static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // See the "Follow a single path" example for more info on what gets passed here
  Command pathfindingCommand =
      new PathfindHolonomic(
          targetPose,
          constraints,
          0.0, // Goal end velocity in m/s. Optional
          swerve::getPose,
          swerve::getRobotRelativeSpeeds,
          (speeds) -> swerve.requestVelocity(speeds, false),
          new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
              // in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options
              // here
              ),
          0.0, // Rotation delay distance in meters. This is how far the robot should travel before
          // attempting to rotate. Optional
          swerve // Reference to drive subsystem to set requirements
          );

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

  public Command getAutonomousCommand() {
    return new TrajectoryFollowerCommand(Robot.sixNoteA, swerve, true)
        .andThen(new TrajectoryFollowerCommand(Robot.sixNoteB, swerve))
        .andThen(new TrajectoryFollowerCommand(Robot.sixNoteC, swerve));
  }
}
