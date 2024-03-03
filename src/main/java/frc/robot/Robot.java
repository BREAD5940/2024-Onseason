package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commons.LoggedTunableNumber;
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

  public static PathPlannerPath fourNoteCenterA;
  public static PathPlannerPath fourNoteCenterB;
  public static PathPlannerPath fourNoteCenterC;
  public static PathPlannerPath fourNoteCenterD;

  public static PathPlannerPath fourNoteSourceSideA;
  public static PathPlannerPath fourNoteSourceSideB;
  public static PathPlannerPath fourNoteSourceSideC;
  public static PathPlannerPath fourNoteSourceSideD;
  public static PathPlannerPath fourNoteSourceSideE;
  public static PathPlannerPath fourNoteSourceSideF;

  public static PathPlannerPath sixNoteAmpSideA;
  public static PathPlannerPath sixNoteAmpSideB;
  public static PathPlannerPath sixNoteAmpSideC;
  public static PathPlannerPath sixNoteAmpSideD;
  public static PathPlannerPath sixNoteAmpSideE;
  public static PathPlannerPath sixNoteAmpSideF;
  public static PathPlannerPath sixNoteAmpSideG;

  public static PathPlannerPath fiveNoteMidlineAmpSideA;
  public static PathPlannerPath fiveNoteMidlineAmpSideB;
  public static PathPlannerPath fiveNoteMidlineAmpSideC;
  public static PathPlannerPath fiveNoteMidlineAmpSideD;
  public static PathPlannerPath fiveNoteMidlineAmpSideE;
  public static PathPlannerPath fiveNoteMidlineAmpSideF;

  public static PathPlannerPath bombA;
  public static PathPlannerPath bombB;
  public static PathPlannerPath bombC;

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

    fourNoteCenterA = PathPlannerPath.fromPathFile("Four Note Center A");
    fourNoteCenterB = PathPlannerPath.fromPathFile("Four Note Center B");
    fourNoteCenterC = PathPlannerPath.fromPathFile("Four Note Center C");
    fourNoteCenterD = PathPlannerPath.fromPathFile("Four Note Center D");

    fourNoteSourceSideA = PathPlannerPath.fromPathFile("Four Note Source A");
    fourNoteSourceSideB = PathPlannerPath.fromPathFile("Four Note Source B");
    fourNoteSourceSideC = PathPlannerPath.fromPathFile("Four Note Source C");
    fourNoteSourceSideD = PathPlannerPath.fromPathFile("Four Note Source D");
    fourNoteSourceSideE = PathPlannerPath.fromPathFile("Four Note Source E");
    fourNoteSourceSideF = PathPlannerPath.fromPathFile("Four Note Source F");

    sixNoteAmpSideA = PathPlannerPath.fromPathFile("Six Note A");
    sixNoteAmpSideB = PathPlannerPath.fromPathFile("Six Note B");
    sixNoteAmpSideC = PathPlannerPath.fromPathFile("Six Note C");
    sixNoteAmpSideD = PathPlannerPath.fromPathFile("Six Note D");
    sixNoteAmpSideE = PathPlannerPath.fromPathFile("Six Note E");
    sixNoteAmpSideF = PathPlannerPath.fromPathFile("Six Note F");
    sixNoteAmpSideG = PathPlannerPath.fromPathFile("Six Note G");

    fiveNoteMidlineAmpSideA = PathPlannerPath.fromPathFile("Five Note Midline A");
    fiveNoteMidlineAmpSideB = PathPlannerPath.fromPathFile("Five Note Midline B");
    fiveNoteMidlineAmpSideC = PathPlannerPath.fromPathFile("Five Note Midline C");
    fiveNoteMidlineAmpSideD = PathPlannerPath.fromPathFile("Five Note Midline D");
    fiveNoteMidlineAmpSideE = PathPlannerPath.fromPathFile("Five Note Midline E");
    fiveNoteMidlineAmpSideF = PathPlannerPath.fromPathFile("Five Note Midline F");

    bombA = PathPlannerPath.fromPathFile("Bomb A");
    bombB = PathPlannerPath.fromPathFile("Bomb B");
    bombC = PathPlannerPath.fromPathFile("Bomb C");

    m_robotContainer.configureAutonomousSelector();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Controller vibrations
    if (RobotContainer.superstructure.hasPiece()) {
      leds.setLEDs(0, 0, 255, 0, 0, 60);
    } else {
      leds.setLEDs(255, 0, 0, 0, 0, 60);
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
    RobotContainer.shooter.requestIdle();
    RobotController.setBrownoutVoltage(5.75);
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.aprilTagVision.setStdDevMode(StdDevMode.AUTONOMOUS);
  }

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
  }

  @Override
  public void teleopPeriodic() {
    RobotContainer.aprilTagVision.setStdDevMode(StdDevMode.TELEOP);
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
