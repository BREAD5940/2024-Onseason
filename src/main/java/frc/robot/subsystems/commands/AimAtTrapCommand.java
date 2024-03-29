package frc.robot.subsystems.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.swerve.Swerve;

public class AimAtTrapCommand extends Command {

  // Subsystems
  private Swerve swerve;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0);

  public AimAtTrapCommand(Swerve swerve) {
    this.swerve = swerve;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double setpoint = 1.078;
    double measurement = swerve.getPose().getRotation().getRadians();

    double output = turnPID.calculate(measurement, setpoint);

    // Drive Inputs
    double x = BreadUtil.deadband(RobotContainer.driver.getLeftY(), 0.1);
    double y = BreadUtil.deadband(RobotContainer.driver.getLeftX(), 0.1);

    double dx;
    double dy;

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      dx = Math.pow(-x, 1) * 1.0;
      dy = Math.pow(-y, 1) * 1.0;

    } else {
      dx = Math.pow(-x, 1) * -1 * 1.0;
      dy = Math.pow(-y, 1) * -1 * 1.0;
    }

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);
  }
}
