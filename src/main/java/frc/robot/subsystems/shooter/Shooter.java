package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /* IO and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double desiredLeftRPM = 0.0;
  private double desiredRightRPM = 0.0;

  /* System variables and parameters */
  private ShooterState systemState = ShooterState.IDLE;

  private boolean requestFender = false;
  private boolean requestVisionSpeaker = false;
  private boolean requestAmp = false;

  private boolean wantsShootOverDefense = false;

  private double stateStartTime = 0.0;

  /* System states */
  public enum ShooterState {
    IDLE,
    FENDER,
    VISION_SPEAKER,
    AMP
  }

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    /* Handle statemachine loic */
    ShooterState nextSystemState = systemState;

    if (systemState == ShooterState.IDLE) {
      // Outputs
      io.setVelocity(SHOOTER_LEFT_IDLE_RPM, SHOOTER_RIGHT_IDLE_RPM);

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
      io.setVelocity(SHOOTER_LEFT_FENDER_RPM, SHOOTER_RIGHT_FENDER_RPM);

      // Transitions
      if (!requestFender) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.VISION_SPEAKER) {
      // Outputs
      // TODO add output here

      // Transitions
      if (!requestVisionSpeaker) {
        nextSystemState = ShooterState.IDLE;
      }
    } else if (systemState == ShooterState.AMP) {
      io.setVelocity(SHOOTER_LEFT_AMP_RPM, SHOOTER_RIGHT_AMP_RPM);

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
    return false;
  }

  public void requestIdle() {
    unsetAllRequests();
  }

  public void requestFender(boolean wantsShootOverDefense) {
    unsetAllRequests();
    requestFender = true;
    this.wantsShootOverDefense = wantsShootOverDefense;
  }

  public void requestVisionSpeaker(boolean wantsShootOverDefense) {
    unsetAllRequests();
    requestVisionSpeaker = true;
    this.wantsShootOverDefense = wantsShootOverDefense;
  }

  public void requestAmp() {
    unsetAllRequests();
    requestAmp = true;
  }

  private void unsetAllRequests() {
    requestFender = false;
    requestVisionSpeaker = false;
    requestAmp = false;
  }
}
