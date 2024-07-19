package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.Shooter.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /* IO and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /* System variables and parameters */
  private ShooterState systemState = ShooterState.IDLE;

  private boolean requestFender = false;
  private boolean requestVisionSpeaker = false;
  private boolean requestAmp = false;
  private boolean requestPass = false;
  private boolean requestTrap = false;
  private boolean requestLowPass = false;

  private boolean wantsShootOverDefense = false;
  private boolean override = false;

  private double stateStartTime = 0.0;

  /* Setpoints */
  private double desiredLeftRPM = 1000.0;
  private double desiredRightRPM = 1000.0;

  /* Tunable numbers */
  LoggedTunableNumber ampSpeed = new LoggedTunableNumber("Amp/ShotVelocity", SHOOTER_LEFT_AMP_RPM);

  /* System states */
  public enum ShooterState {
    IDLE,
    FENDER,
    VISION_SPEAKER,
    AMP,
    TUNING,
    PASS,
    TRAP,
    LOW_PASS
  }

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/SystemState", systemState);

    /* Handle statemachine logic */
    ShooterState nextSystemState = systemState;

    if (systemState == ShooterState.IDLE) {
      // Outputs
      if (RobotContainer.operator.getLeftTriggerAxis() > 0.1) {
        ShotParameter shot = RobotContainer.visionSupplier.robotToSpeakerShot();
        io.setVelocity(shot.leftRPM, shot.rightRPM);
      } else {
        io.setVelocity(desiredLeftRPM, desiredRightRPM);
      }

      // Transitions
      if (requestFender) {
        nextSystemState = ShooterState.FENDER;
      } else if (requestVisionSpeaker) {
        nextSystemState = ShooterState.VISION_SPEAKER;
      } else if (requestAmp) {
        nextSystemState = ShooterState.AMP;
      } else if (requestPass) {
        nextSystemState = ShooterState.PASS;
      } else if (requestTrap) {
        nextSystemState = ShooterState.TRAP;
      } else if (requestLowPass) {
        nextSystemState = ShooterState.LOW_PASS;
      }
    } else if (systemState == ShooterState.FENDER) {
      // Outputs
      io.setVelocity(desiredLeftRPM, desiredRightRPM);

      // Transitions
      if (!requestFender) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.VISION_SPEAKER) {
      // Outputs
      io.setVelocity(desiredLeftRPM, desiredRightRPM);

      // Transitions
      if (!requestVisionSpeaker) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.AMP) {
      io.setVelocity(desiredLeftRPM, desiredRightRPM);

      if (!requestAmp) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.PASS) {
      io.setVelocity(desiredLeftRPM, desiredRightRPM);

      if (!requestPass) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.TRAP) {
      io.setPercent(desiredLeftRPM, desiredRightRPM);

      if (!requestTrap) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.LOW_PASS) {
      io.setVelocity(desiredLeftRPM, desiredRightRPM);

      if (!requestLowPass) {
        nextSystemState = ShooterState.IDLE;
      }
    }

    if (nextSystemState != systemState) {
      stateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
  }

  /* Returns whether or not the shooter is spinning */
  public boolean shooterNotSpinning() {
    return Math.abs(inputs.shooterLeftVelocityRpm) < 10.0
        && Math.abs(inputs.shooterRightVelocityRpm) < 10.0;
  }

  // Returns whether the shooter is at setpoint for the superstructure
  public boolean atSetpoint() {
    Logger.recordOutput("Shooter error 1", desiredLeftRPM - inputs.shooterLeftVelocityRpm);
    Logger.recordOutput("Shooter error 2", desiredRightRPM - inputs.shooterRightVelocityRpm);
    return Math.abs(desiredLeftRPM - inputs.shooterLeftVelocityRpm) < SHOOTER_SETPOINT_TOLERANCE_RPM
        && Math.abs(desiredRightRPM - inputs.shooterRightVelocityRpm)
            < SHOOTER_SETPOINT_TOLERANCE_RPM;
  }

  public void requestIdle() {
    desiredLeftRPM = SHOOTER_LEFT_IDLE_RPM;
    desiredRightRPM = SHOOTER_RIGHT_IDLE_RPM;
    unsetAllRequests();
  }

  public void requestFender() {
    desiredLeftRPM = Robot.leftSpeed.get();
    desiredRightRPM = Robot.rightSpeed.get();
    // desiredLeftRPM = SHOOTER_LEFT_FENDER_RPM;
    // desiredRightRPM = SHOOTER_RIGHT_FENDER_RPM;
    unsetAllRequests();
    requestFender = true;
  }

  public void requestVisionSpeaker(boolean wantsShootOverDefense) {
    ShotParameter shot;
    if (wantsShootOverDefense) {
      shot = RobotContainer.visionSupplier.robotToSpeakerShotSOD();
    } else {
      shot = RobotContainer.visionSupplier.robotToSpeakerShot();
    }
    desiredLeftRPM = shot.leftRPM;
    desiredRightRPM = shot.rightRPM;
    // desiredLeftRPM = Robot.leftSpeed.get();
    // desiredRightRPM = Robot.rightSpeed.get();
    unsetAllRequests();
    requestVisionSpeaker = true;
    this.wantsShootOverDefense = wantsShootOverDefense;
    this.override = true;
  }

  public void requestSpeakerOverride(double leftSpeed, double rightSpeed) {
    desiredLeftRPM = leftSpeed;
    desiredRightRPM = rightSpeed;
    unsetAllRequests();
    requestVisionSpeaker = true;
    this.override = false;
  }

  public void requestAmp() {
    desiredLeftRPM = ampSpeed.get();
    desiredRightRPM = ampSpeed.get();
    unsetAllRequests();
    requestAmp = true;
  }

  public void requestPass() {
    ShotParameter shot = RobotContainer.visionSupplier.robotToPassingShot();
    if (Robot.alliance == Alliance.Red) {
      desiredLeftRPM = shot.rightRPM;
      desiredRightRPM = shot.leftRPM;
    } else {
      desiredLeftRPM = shot.leftRPM;
      desiredRightRPM = shot.rightRPM;
    }
    unsetAllRequests();
    requestPass = true;
  }

  public void requestTrap() {
    desiredLeftRPM = -0.25;
    desiredRightRPM = -0.25;
    unsetAllRequests();
    requestTrap = true;
  }

  public void requestLowPass() {
    desiredLeftRPM = SHOOTER_LEFT_PASS_RPM;
    desiredRightRPM = SHOOTER_RIGHT_PASS_RPM;
    unsetAllRequests();
    requestLowPass = true;
  }

  private void unsetAllRequests() {
    requestFender = false;
    requestVisionSpeaker = false;
    requestAmp = false;
    requestPass = false;
    requestTrap = false;
    requestLowPass = false;
  }
}
