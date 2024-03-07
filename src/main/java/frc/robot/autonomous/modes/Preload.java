package frc.robot.autonomous.modes;

import static frc.robot.constants.Constants.AutoShots.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class Preload extends SequentialCommandGroup {

  public Preload(Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        // Fire preload
        new InstantCommand(() -> superstructure.requestIntake(true)),
        new WaitUntilCommand(() -> superstructure.hasPiece()),
        new InstantCommand(() -> superstructure.requestIntake(false)),

        // Drive back and shoot
        new InstantCommand(() -> swerve.requestVelocity(new ChassisSpeeds(-1.0, 0.0, 0.0), false)),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(3.0),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }
}
