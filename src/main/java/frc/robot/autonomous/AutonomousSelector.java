package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.Bomb;
import frc.robot.autonomous.modes.CheekyThreePiece;
import frc.robot.autonomous.modes.DriveBackTwo;
import frc.robot.autonomous.modes.FiveNoteAmpSide;
import frc.robot.autonomous.modes.FourNoteAmpSide;
import frc.robot.autonomous.modes.FourNoteCenter;
import frc.robot.autonomous.modes.FourNoteSourceSide;
import frc.robot.autonomous.modes.OPAuto;
import frc.robot.autonomous.modes.Preload;
import frc.robot.autonomous.modes.ReverseFiveNoteAmpSide;
import frc.robot.autonomous.modes.SixNoteAmpSide;
import frc.robot.autonomous.modes.SixNoteAmpSideAlternate;
import frc.robot.autonomous.modes.SourceSideRush12;
import frc.robot.autonomous.modes.SourceSideRush21;
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
        "CHEEKY_THREE_PIECE", new CheekyThreePiece(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "SIX_NOTE_AMP_SIDE", new SixNoteAmpSide(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "SIX_NOTE_AMP_SIDE_ALTERNATE",
        new SixNoteAmpSideAlternate(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "FOUR_NOTE_CENTER", new FourNoteCenter(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption("BOMB", new Bomb(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "FOUR_NOTE_AMP_SIDE", new FourNoteAmpSide(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "FIVE_NOTE_AMP_SIDE", new FiveNoteAmpSide(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption("PRELOAD", new Preload(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "DRIVE_BACK_TWO", new DriveBackTwo(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "FOUR_NOTE_SOURCE_SIDE", new FourNoteSourceSide(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "REVERSE_FIVE_NOTE", new ReverseFiveNoteAmpSide(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption("OP_AUTO", new OPAuto(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "SOURCE_SIDE_RUSH_12", new SourceSideRush12(superstructure, swerve, shooter, intake));
    autonomousSelector.addOption(
        "SOURCE_SIDE_RUSH_21", new SourceSideRush21(superstructure, swerve, shooter, intake));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
