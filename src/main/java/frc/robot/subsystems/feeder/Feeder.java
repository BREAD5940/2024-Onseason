package frc.robot.subsystems.feeder;

import static frc.robot.constants.Constants.Feeder.*;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commons.BreadUtil;
import org.littletonrobotics.junction.Logger;

public class Feeder {

  // Hardware
  private FeederIO io;
  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  // System variables and parameters
  private FeederState systemState = FeederState.IDLE;

  private boolean requestIntake = false;
  private boolean requestSpit = false;
  private boolean requestShoot = false;

  private boolean hasPiece = false;

  private Timer shootingTimer = new Timer();

  private double stateStartTime = 0.0;

  // System States
  public enum FeederState {
    IDLE,
    INTAKE,
    SPIT,
    SHOOT
  }

  public Feeder(FeederIO io) {
    this.io = io;
    shootingTimer.reset();
    shootingTimer.start();
  }

  /* Method to be periodically by the superstructure */
  public void onLoop() {
    // Log
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    // Handle statemachine logic
    FeederState nextSystemState = systemState;
    if (systemState == FeederState.IDLE) {
      io.setVelocity(0.0);

      if (requestIntake && !hasPiece) {
        nextSystemState = FeederState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = FeederState.SPIT;
      } else if (requestShoot && hasPiece) {
        shootingTimer.start();
        nextSystemState = FeederState.SHOOT;
      }
    } else if (systemState == FeederState.INTAKE) {
      io.setVelocity(FEEDER_INTAKE_SPEED);

      if (!requestIntake) {
        nextSystemState = FeederState.IDLE;
      } else if (inputs.beamBreakTriggered) {
        hasPiece = true;
        nextSystemState = FeederState.IDLE;
      }
    } else if (systemState == FeederState.SPIT) {
      io.setVelocity(FEEDER_SPIT_SPEED);

      if (!requestSpit) {
        hasPiece = false;
        nextSystemState = FeederState.IDLE;
      }
    } else if (systemState == FeederState.SHOOT) {
      io.setVelocity(FEEDER_SHOOT_SPEED);

      if (!requestShoot) {
        nextSystemState = FeederState.IDLE;
      }
      if (!inputs.beamBreakTriggered && shootingTimer.get() > 2.0) {
        hasPiece = false;
        nextSystemState = FeederState.IDLE;
      }
    }

    if (nextSystemState != systemState) {
      stateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
  }

  public boolean hasPiece() {
    return hasPiece;
  }

  public FeederState getSystemState() {
    return systemState;
  }

  public void requestIdle() {
    unsetAllRequests();
  }

  public void requestIntake() {
    unsetAllRequests();
    requestIntake = true;
  }

  public void requestSpit() {
    unsetAllRequests();
    requestSpit = true;
  }

  public void requestShoot() {
    unsetAllRequests();
    requestShoot = true;
  }

  private void unsetAllRequests() {
    requestIntake = false;
    requestSpit = false;
    requestShoot = false;
  }
}
