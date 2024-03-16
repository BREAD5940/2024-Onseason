package frc.robot.subsystems.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutoPickupNote extends Command {

  Swerve swerve;
  Superstructure superstructure;

  private PIDController turnPID = new PIDController(7, 0, 0);
  private PIDController xPID = new PIDController(1, 0, 0);
  private PIDController yPID = new PIDController(1, 0, 0);

  public AutoPickupNote(Swerve swerve, Superstructure superstructure) {
    this.swerve = swerve;
    this.superstructure = superstructure;

    addRequirements(swerve, superstructure);

    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    RobotContainer.intake.requestIntake();
    superstructure.requestIntake(true);

    double turnSetpoint = RobotContainer.visionSupplier.robotToNoteAngle().getRadians();
    double turnMeasurement = swerve.getPose().getRotation().getRadians();
    double turnOutput = turnPID.calculate(turnMeasurement, turnSetpoint);

    double xSetpoint = RobotContainer.visionSupplier.notePose().getX();
    double xMeasurement = swerve.getPose().getX();
    double xOutput = MathUtil.clamp(xPID.calculate(xMeasurement, xSetpoint), -1, 1);

    double ySetpoint = RobotContainer.visionSupplier.notePose().getY();
    double yMeasurement = swerve.getPose().getY();
    double yOutput = MathUtil.clamp(yPID.calculate(yMeasurement, ySetpoint), -1, 1);

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(xOutput, yOutput, turnOutput), true);
  }

  @Override
  public boolean isFinished() {
    return superstructure.hasPiece();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), false);
    superstructure.requestIntake(false);
    RobotContainer.intake.requestIdle();
  }
}
