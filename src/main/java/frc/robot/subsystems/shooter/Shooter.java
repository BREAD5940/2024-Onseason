package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    TUNING
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

    /* Handle statemachine loic */
    ShooterState nextSystemState = systemState;

    if (systemState == ShooterState.IDLE) {
      // Outputs
      io.setVelocity(desiredLeftRPM, desiredRightRPM);

      // Transitions
      if (requestFender) {
        nextSystemState = ShooterState.FENDER;
      } else if (requestVisionSpeaker) {
        nextSystemState = ShooterState.VISION_SPEAKER;
      } else if (requestAmp) {
        nextSystemState = ShooterState.AMP;
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
    }

    if (nextSystemState != systemState) {
      stateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
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
    // desiredLeftRPM = Robot.leftSpeed.get();
    // desiredRightRPM = Robot.rightSpeed.get();
    desiredLeftRPM = SHOOTER_LEFT_FENDER_RPM;
    desiredRightRPM = SHOOTER_RIGHT_FENDER_RPM;
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

  private void unsetAllRequests() {
    requestFender = false;
    requestVisionSpeaker = false;
    requestAmp = false;
  }
}
