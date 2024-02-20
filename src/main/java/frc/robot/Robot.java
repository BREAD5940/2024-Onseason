package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.vision.photonvision.PhotonAprilTagVision.StdDevMode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static PathPlannerPath threeNoteCenterA;
  public static PathPlannerPath threeNoteCenterB;
  public static PathPlannerPath threeNoteCenterC;

  public static PathPlannerPath fourNoteA;
  public static PathPlannerPath fourNoteB;
  public static PathPlannerPath fourNoteC;
  public static PathPlannerPath fourNoteD;
  public static PathPlannerPath fourNoteE;
  public static PathPlannerPath fourNoteF;

  public static PathPlannerPath fiveNoteA;
  public static PathPlannerPath fiveNoteB;
  public static PathPlannerPath fiveNoteC;
  public static PathPlannerPath fiveNoteD;
  public static PathPlannerPath fiveNoteE;

  private boolean requestedHome = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", "2024-Beta"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start();

    threeNoteCenterA = PathPlannerPath.fromPathFile("Three Note Center A");
    threeNoteCenterB = PathPlannerPath.fromPathFile("Three Note Center B");
    threeNoteCenterC = PathPlannerPath.fromPathFile("Three Note Center C");

    fourNoteA = PathPlannerPath.fromPathFile("Four Note A");
    fourNoteB = PathPlannerPath.fromPathFile("Four Note B");
    fourNoteC = PathPlannerPath.fromPathFile("Four Note C");
    fourNoteD = PathPlannerPath.fromPathFile("Four Note D");
    fourNoteE = PathPlannerPath.fromPathFile("Four Note E");
    fourNoteF = PathPlannerPath.fromPathFile("Four Note F");

    fiveNoteA = PathPlannerPath.fromPathFile("Five Note A");
    fiveNoteB = PathPlannerPath.fromPathFile("Five Note B");
    fiveNoteC = PathPlannerPath.fromPathFile("Five Note C");
    fiveNoteD = PathPlannerPath.fromPathFile("Five Note D");
    fiveNoteE = PathPlannerPath.fromPathFile("Five Note E");

    m_robotContainer.configureAutonomousSelector();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Set std dev scalar based on superstructure state
    if (RobotContainer.superstructure.getSystemState() == SuperstructureState.VISION_SPEAKER) {
      RobotContainer.aprilTagVision.setStdDevMode(StdDevMode.SHOOTING);
    } else {
      RobotContainer.aprilTagVision.setStdDevMode(StdDevMode.DEFAULT);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (!requestedHome) {
      RobotContainer.superstructure.requestHome();
      requestedHome = true;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!requestedHome) {
      RobotContainer.superstructure.requestHome();
      requestedHome = true;
    }
  }

  @Override
  public void teleopPeriodic() {
    /* Intake requests */
    if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
      RobotContainer.intake.requestIntake();
      RobotContainer.superstructure.requestIntake(true);

      RobotContainer.superstructure.requestIntake(true);
    } else if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
      RobotContainer.intake.requestSpit();
    } else {
      RobotContainer.superstructure.requestIntake(false);
      RobotContainer.intake.requestIdle();
    }

    /* Superstructure spit requests */

    // Controller vibrations
    if (RobotContainer.intake.hasPiece()) {
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0.0);
    }

    /* Climb requests */
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // double targetHeight = testHeight.get();
    // Rotation2d targetAngle = Rotation2d.fromDegrees(testAngle.get());

    // RobotContainer.superstructure.elevatorPivot.requestPursueSetpoint(targetAngle, targetHeight);
  }

  @Override
  public void testExit() {}
}
