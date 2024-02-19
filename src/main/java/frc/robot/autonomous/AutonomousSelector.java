package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.FiveNote;
import frc.robot.autonomous.modes.FourNote;
import frc.robot.autonomous.modes.ThreeNoteCenter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  public AutonomousSelector(
      Swerve swerve, Superstructure superstructure, Shooter shooter, Intake intake) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
    autonomousSelector.addOption(
        "THREE_NOTE_CENTER", new ThreeNoteCenter(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "FOUR_NOTE", new FourNote(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "FIVE_NOTE", new FiveNote(superstructure, swerve, shooter, intake));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
