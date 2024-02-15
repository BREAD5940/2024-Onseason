package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.commands.StationaryShootCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class ThreeNoteCenter extends SequentialCommandGroup {

  public ThreeNoteCenter(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(3),
        new TrajectoryFollowerCommand(Robot.threeNoteCenterA, swerve, true)
            .beforeStarting(intake::requestIntake),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.threeNoteCenterB, swerve, false),
        new StationaryShootCommand(swerve, superstructure, shooter).withTimeout(2),
        new TrajectoryFollowerCommand(Robot.threeNoteCenterC, swerve, false)
            .beforeStarting(intake::requestIntake),
        new WaitUntilCommand(() -> superstructure.hasPiece()).withTimeout(2),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }
}