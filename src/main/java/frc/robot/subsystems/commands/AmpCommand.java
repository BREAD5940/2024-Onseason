package frc.robot.subsystems.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class AmpCommand extends Command {

  // Subsystems
  private Swerve swerve;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0);

  public AmpCommand(Swerve swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // Calculate swerve outputs
    double setpoint = Math.PI / 2.0;
    double measurement = swerve.getPose().getRotation().getRadians();

    double output = turnPID.calculate(measurement, setpoint);

    double x = BreadUtil.deadband(RobotContainer.driver.getLeftY(), 0.1);
    double y = BreadUtil.deadband(RobotContainer.driver.getLeftX(), 0.1);
    double omega = BreadUtil.deadband(RobotContainer.driver.getRightX(), 0.1);

    double dx;
    double dy;

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      dx = Math.pow(-x, 1) * 2.0;
      dy = Math.pow(-y, 1) * 2.0;

    } else {
      dx = Math.pow(-x, 1) * -1 * 2.0;
      dy = Math.pow(-y, 1) * -1 * 2.0;
    }

    // Apply swerve Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);

    RobotContainer.superstructure.requestAmp(true, RobotContainer.driver.getYButton());
    RobotContainer.shooter.requestAmp();

    // Apply superstructure requests
    // if (Math.abs(setpoint - measurement) < Units.degreesToRadians(15.0)) {
    //   RobotContainer.superstructure.requestAmp(true, RobotContainer.driver.getYButton());
    //   RobotContainer.shooter.requestAmp();
    // } else {
    //   RobotContainer.superstructure.requestAmp(false, false);
    //   RobotContainer.shooter.requestIdle();
    // }

    Logger.recordOutput(
        "AimAtSpeakerCommand/measurementDegrees", Units.radiansToDegrees(measurement));
    Logger.recordOutput("AimAtSpeakerCommand/setpointDegrees", Units.radiansToDegrees(setpoint));
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.superstructure.requestAmp(false, false);
    RobotContainer.shooter.requestIdle();
  }
}
