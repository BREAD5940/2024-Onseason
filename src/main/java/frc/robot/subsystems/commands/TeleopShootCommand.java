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

public class TeleopShootCommand extends Command {

  // Subsystems
  private Swerve swerve;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0);

  public TeleopShootCommand(Swerve swerve) {
    this.swerve = swerve;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Swerve turn PID calcs
    double setpoint = RobotContainer.visionSupplier.robotToSpeakerAngle().getRadians();
    double measurement = swerve.getPose().getRotation().getRadians();

    double output = turnPID.calculate(measurement, setpoint);

    // Drive Inputs
    double x = BreadUtil.deadband(RobotContainer.driver.getLeftY(), 0.1);
    double y = BreadUtil.deadband(RobotContainer.driver.getLeftX(), 0.1);

    double dx;
    double dy;

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      dx = Math.pow(-x, 1) * 1.5;
      dy = Math.pow(-y, 1) * 1.5;

    } else {
      dx = Math.pow(-x, 1) * -1 * 1.5;
      dy = Math.pow(-y, 1) * -1 * 1.5;
    }

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);
    // RobotContainer.superstructure.requestFender(true, RobotContainer.driver.getYButton());
    // RobotContainer.shooter.requestFender();
    RobotContainer.superstructure.requestVisionSpeaker(
        true,
        RobotContainer.swerve.atAngularSetpoint(setpoint) && RobotContainer.driver.getYButton(),
        RobotContainer.operator.getYButton());
    RobotContainer.shooter.requestVisionSpeaker(false);

    // Logs
    Logger.recordOutput(
        "TeleopShootCommand/MeasurementDegrees", Units.radiansToDegrees(measurement));
    Logger.recordOutput("TeleopShootCommand/SetpointDegrees", Units.radiansToDegrees(setpoint));
    Logger.recordOutput(
        "TeleopShootCommand/atTurnSetpoint", RobotContainer.swerve.atAngularSetpoint(setpoint));
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.requestIdle();
    RobotContainer.superstructure.requestVisionSpeaker(false, false, false);
    // RobotContainer.superstructure.requestFender(false, false);
  }
}
