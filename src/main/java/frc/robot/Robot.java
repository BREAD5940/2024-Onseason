package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static Alliance alliance = DriverStation.Alliance.Red;

  public static PathPlannerPath fourNoteCenterA;
  public static PathPlannerPath fourNoteCenterB;
  public static PathPlannerPath fourNoteCenterC;
  public static PathPlannerPath fourNoteCenterD;

  public static PathPlannerPath cheekyThreePieceA;
  public static PathPlannerPath cheekyThreePieceB;

  public static PathPlannerPath fourNoteSourceSideA;
  public static PathPlannerPath fourNoteSourceSideB;
  public static PathPlannerPath fourNoteSourceSideC;

  public static PathPlannerPath sixNoteAmpSideA;
  public static PathPlannerPath sixNoteAmpSideB;
  public static PathPlannerPath sixNoteAmpSideC;
  public static PathPlannerPath sixNoteAmpSideD;
  public static PathPlannerPath sixNoteAmpSideE;
  public static PathPlannerPath sixNoteAmpSideF;

  public static PathPlannerPath fiveNoteMidlineAmpSideA;
  public static PathPlannerPath fiveNoteMidlineAmpSideB;
  public static PathPlannerPath fiveNoteMidlineAmpSideC;
  public static PathPlannerPath fiveNoteMidlineAmpSideD;
  public static PathPlannerPath fiveNoteMidlineAmpSideE;
  public static PathPlannerPath fiveNoteMidlineAmpSideF;

  public static PathPlannerPath reverseSixNoteARed;
  public static PathPlannerPath reverseSixNoteBRed;
  public static PathPlannerPath reverseSixNoteCRed;

  public static PathPlannerPath reverseSixNoteABlue;
  public static PathPlannerPath reverseSixNoteBBlue;
  public static PathPlannerPath reverseSixNoteCBlue;

  public static PathPlannerPath bombA;
  public static PathPlannerPath bombB;
  public static PathPlannerPath bombC;

  public static PathPlannerPath fiveNoteAmpSideA;
  public static PathPlannerPath fiveNoteAmpSideB;
  public static PathPlannerPath fiveNoteAmpSideC;
  public static PathPlannerPath fiveNoteAmpSideD;
  public static PathPlannerPath fiveNoteAmpSideAlternate;

  private boolean requestedHome = false;

  private CANdle leds = new CANdle(31, "dabus");

  public static LoggedTunableNumber leftSpeed = new LoggedTunableNumber("Tuning/LeftSpeed", 0.0);
  public static LoggedTunableNumber rightSpeed = new LoggedTunableNumber("Tuning/RightSpeed", 0.0);

  public static LoggedTunableNumber elevatorHeight =
      new LoggedTunableNumber("Tuning/ElevatorHeight", 0.0);
  public static LoggedTunableNumber pivotAngle =
      new LoggedTunableNumber("Tuning/PivotAngleDegrees", 0.0);

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
      RobotController.setBrownoutVoltage(5.75);
    }

    Logger.start();
    Logger.disableDeterministicTimestamps();
    Logger.disableConsoleCapture();

    fourNoteCenterA = PathPlannerPath.fromPathFile("Four Note Center A");
    fourNoteCenterB = PathPlannerPath.fromPathFile("Four Note Center B");
    fourNoteCenterC = PathPlannerPath.fromPathFile("Four Note Center C");
    fourNoteCenterD = PathPlannerPath.fromPathFile("Four Note Center D");

    cheekyThreePieceA = PathPlannerPath.fromPathFile("Cheeky Three Piece A");
    cheekyThreePieceB = PathPlannerPath.fromPathFile("Cheeky Three Piece B");

    sixNoteAmpSideA = PathPlannerPath.fromPathFile("Six Note A");
    sixNoteAmpSideB = PathPlannerPath.fromPathFile("Six Note B");
    sixNoteAmpSideC = PathPlannerPath.fromPathFile("Six Note C");
    sixNoteAmpSideD = PathPlannerPath.fromPathFile("Six Note D");
    sixNoteAmpSideE = PathPlannerPath.fromPathFile("Six Note E");
    sixNoteAmpSideF = PathPlannerPath.fromPathFile("Six Note F");

    fiveNoteMidlineAmpSideA = PathPlannerPath.fromPathFile("Five Note Midline A");
    fiveNoteMidlineAmpSideB = PathPlannerPath.fromPathFile("Five Note Midline B");
    fiveNoteMidlineAmpSideC = PathPlannerPath.fromPathFile("Five Note Midline C");
    fiveNoteMidlineAmpSideD = PathPlannerPath.fromPathFile("Five Note Midline D");
    fiveNoteMidlineAmpSideE = PathPlannerPath.fromPathFile("Five Note Midline E");
    fiveNoteMidlineAmpSideF = PathPlannerPath.fromPathFile("Five Note Midline F");

    reverseSixNoteARed = PathPlannerPath.fromPathFile("Reverse Six Note A Red");
    reverseSixNoteBRed = PathPlannerPath.fromPathFile("Reverse Six Note B Red");

    reverseSixNoteABlue = PathPlannerPath.fromPathFile("Reverse Six Note A Blue");
    reverseSixNoteBBlue = PathPlannerPath.fromPathFile("Reverse Six Note B Blue");

    bombA = PathPlannerPath.fromPathFile("Bomb A");
    bombB = PathPlannerPath.fromPathFile("Bomb B");
    bombC = PathPlannerPath.fromPathFile("Bomb C");

    fiveNoteAmpSideA = PathPlannerPath.fromPathFile("Five Note Amp Side A");
    fiveNoteAmpSideB = PathPlannerPath.fromPathFile("Five Note Amp Side B");
    fiveNoteAmpSideC = PathPlannerPath.fromPathFile("Five Note Amp Side C");
    fiveNoteAmpSideD = PathPlannerPath.fromPathFile("Five Note Amp Side D");
    fiveNoteAmpSideAlternate = PathPlannerPath.fromPathFile("Five Note Amp Side Alternate");

    fourNoteSourceSideA = PathPlannerPath.fromPathFile("Four Note Source Side A");
    fourNoteSourceSideB = PathPlannerPath.fromPathFile("Four Note Source Side B");
    fourNoteSourceSideC = PathPlannerPath.fromPathFile("Four Note Source Side C");

    m_robotContainer.configureAutonomousSelector();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // LED colors

    // g r b
    // if (RobotContainer.intake.hasPiece() && RobotContainer.superstructure.hasPiece()) {
    //   leds.setLEDs(255, 255, 0, 0, 0, 60);
    // } else if (RobotContainer.intake.hasPiece()) {
    //   leds.setLEDs(255, 0, 0, 0, 0, 60);
    // } else if (RobotContainer.superstructure.hasPiece()) {
    //   leds.setLEDs(0, 255, 0, 0, 0, 60);
    // } else {
    //   leds.setLEDs(0, 0, 0, 0, 0, 60);
    // }
    if (RobotContainer.superstructure.getSystemState() == SuperstructureState.PRE_CLIMB) {
      leds.setLEDs(255, 0, 0, 0, 0, 60);
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
    } else if (RobotContainer.superstructure.hasPiece()) {
      leds.setLEDs(0, 255, 0, 0, 0, 60);
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0.25);
    } else {
      leds.setLEDs(0, 0, 0, 0, 0, 60);
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
    }

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      alliance = allianceOptional.get();
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
    RobotContainer.superstructure.registerAutoPreload();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (!requestedHome) {
      RobotContainer.superstructure.requestHome();
      requestedHome = true;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.shooter.requestIdle();
    RobotController.setBrownoutVoltage(5.75);
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

    RobotContainer.shooter.requestIdle();
    RobotController.setBrownoutVoltage(5.75);

    RobotContainer.superstructure.requestVisionSpeaker(false, false, false);
  }

  @Override
  public void teleopPeriodic() {
    /* Intake requests */
    if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
      RobotContainer.intake.requestIntake();
      RobotContainer.superstructure.requestIntake(true);
    } else if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
      RobotContainer.intake.requestSpit();
    } else {
      RobotContainer.superstructure.requestIntake(false);
      RobotContainer.intake.requestIdle();
    }

    /* Superstructure spit requests */
    if (RobotContainer.operator.getXButton()) {
      RobotContainer.superstructure.requestSpit(true);
    } else {
      RobotContainer.superstructure.requestSpit(false);
    }

    /* Climb requests */
    if (RobotContainer.operator.getRightBumperPressed()) {
      RobotContainer.superstructure.requestNextClimbState();
    } else if (RobotContainer.operator.getLeftBumperPressed()) {
      RobotContainer.superstructure.requestPrevClimbState();
    }
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
