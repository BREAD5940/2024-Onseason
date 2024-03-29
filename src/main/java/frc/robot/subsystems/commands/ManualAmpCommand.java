package frc.robot.subsystems.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.swerve.Swerve;

public class ManualAmpCommand extends Command {

  // Subsystems
  private Swerve swerve;

  public ManualAmpCommand(Swerve swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double x = BreadUtil.deadband(RobotContainer.driver.getLeftY(), 0.1);
    double y = BreadUtil.deadband(RobotContainer.driver.getLeftX(), 0.1);
    double omega = BreadUtil.deadband(RobotContainer.driver.getRightX(), 0.1);

    double dx;
    double dy;

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      dx = Math.pow(-x, 1) * 2.0;
      dy = Math.pow(-y, 1) * 2.0;

    } else {
      dx = Math.pow(-x, 1) * -1 * 2.0;
      dy = Math.pow(-y, 1) * -1 * 2.0;
    }

    double rot = Math.pow(-omega, 3) * 12.0;

    // Apply swerve Requests
    swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

    RobotContainer.superstructure.requestAmp(true, RobotContainer.driver.getYButton());
    RobotContainer.shooter.requestAmp();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.superstructure.requestAmp(false, false);
    RobotContainer.shooter.requestIdle();
  }
}
