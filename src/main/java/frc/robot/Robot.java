package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commons.LoggedTunableNumber;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static PathPlannerPath sixNoteA;
  public static PathPlannerPath sixNoteB;
  public static PathPlannerPath sixNoteC;

  private boolean requestedHome = false;

  private LoggedTunableNumber testHeight =
      new LoggedTunableNumber("TestMode/ElevatorSetpoint", 0.0);
  private LoggedTunableNumber testAngle = new LoggedTunableNumber("TestMode/PivotAngle", 0.0);

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

    sixNoteA = PathPlannerPath.fromPathFile("Six Note A");
    sixNoteB = PathPlannerPath.fromPathFile("Six Note B");
    sixNoteC = PathPlannerPath.fromPathFile("Six Note C");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    } else if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
      RobotContainer.intake.requestSpit();
    } else {
      RobotContainer.superstructure.requestIntake(false);
      RobotContainer.intake.requestIdle();
    }

    /* Shooting Requests */
    if (RobotContainer.driver.getAButton()) {
      RobotContainer.shooter.requestAmp();
      RobotContainer.superstructure.requestAmp(true, RobotContainer.driver.getBButton());
    } else if (RobotContainer.driver.getBButton()) {
      boolean wantsShootOverDefense = RobotContainer.operator.getAButton();
      RobotContainer.shooter.requestVisionSpeaker(wantsShootOverDefense);
      RobotContainer.superstructure.requestVisionSpeaker(wantsShootOverDefense);
    } else {
      RobotContainer.shooter.requestIdle();
      RobotContainer.superstructure.requestAmp(false, false);
    }

    /* Superstructure spit requests */

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

  private void configureTeleopControls() {}
}
