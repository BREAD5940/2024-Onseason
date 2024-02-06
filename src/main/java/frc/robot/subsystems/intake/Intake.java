package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;

import static frc.robot.constants.Constants.Intake.*;

public class Intake extends SubsystemBase {

  /* IO and input classes */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /* System variables and parameters */
  private IntakeState systemState = IntakeState.IDLE;

  private boolean requestIntake = false;
  private boolean requestSpit = false;

  private boolean hasPiece = false;

  private double stateStartTime = 0.0;

  /* System States */
  public enum IntakeState {
    IDLE,
    INTAKE,
    SPIT
  }

  public Intake(IntakeIO io) {
    this.io = io;
    io.enableBrakeMode(false);
  }

  @Override
  public void periodic() {
    // Logs
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Handle statemachine logic
    IntakeState nextSystemState = systemState;
    if (systemState == IntakeState.IDLE) {
      io.setPercent(0);

      if (requestIntake && !hasPiece) {
        nextSystemState = IntakeState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = IntakeState.SPIT;
      }
    } else if (systemState == IntakeState.INTAKE) {
      io.setPercent(INTAKE_SPEED);

      if (!requestIntake) {
        nextSystemState = IntakeState.IDLE;
      } else if (inputs.topBeamBreakTriggered) {
        hasPiece = true;
        nextSystemState = IntakeState.IDLE;
      }
    } else if (systemState == IntakeState.SPIT) {
      io.setPercent(SPIT_SPEED);

      if (!requestSpit) {
        hasPiece = false;
        nextSystemState = IntakeState.IDLE;
      }
    }

    if (systemState != nextSystemState) {
      stateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
  }

  public boolean hasPiece() {
    return hasPiece;
  }

  public IntakeState getSystemState() {
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

  private void unsetAllRequests() {
    requestIntake = false;
    requestSpit = false;
  }

}
