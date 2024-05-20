package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class PassCommand extends Command {

  private Swerve swerve;
  private Superstructure superstructure;
  private Shooter shooter;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0.2);

  public PassCommand(Swerve swerve, Superstructure superstructure, Shooter shooter) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.shooter = shooter;

    addRequirements(swerve, superstructure, shooter);

    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    // Swerve turn PID calcs
    double setpoint = RobotContainer.visionSupplier.robotToPassingAngle().getRadians();
    double measurement = swerve.getPose().getRotation().getRadians();

    double output = turnPID.calculate(measurement, setpoint);

    // Drive Inputs
    double x = BreadUtil.deadband(RobotContainer.driver.getLeftY(), 0.1);
    double y = BreadUtil.deadband(RobotContainer.driver.getLeftX(), 0.1);

    double dx;
    double dy;

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      dx = Math.pow(-x, 1) * 5;
      dy = Math.pow(-y, 1) * 5;

    } else {
      dx = Math.pow(-x, 1) * -1 * 5;
      dy = Math.pow(-y, 1) * -1 * 5;
    }

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);

    RobotContainer.superstructure.requestPass(
        true, RobotContainer.swerve.atAngularSetpoint(setpoint));
    RobotContainer.shooter.requestPass();
    // RobotContainer.shooter.requestFender();
    // RobotContainer.superstructure.requestFender(
    //     true, RobotContainer.swerve.atAngularSetpoint(setpoint));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle();
    superstructure.requestPass(false, false);
    // superstructure.requestFender(false, false);
  }
}
