package frc.robot.subsystems.intake;

import static frc.robot.constants.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  /* IO and input classes */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /* System variables and parameters */
  private IntakeState systemState = IntakeState.IDLE;

  private boolean requestIntake = false;
  private boolean requestSpit = false;

  private double stateStartTime = 0.0;

  /* System States */
  public enum IntakeState {
    IDLE,
    INTAKE,
    SPIT
  }

  public Intake(IntakeIO io) {
    this.io = io;
    io.enableIntakeBrakeMode(false);
  }

  @Override
  public void periodic() {
    // Logs
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/SystemState", systemState);

    // Handle statemachine logic
    IntakeState nextSystemState = systemState;
    if (systemState == IntakeState.IDLE) {
      io.setIntakePercent(0);
      io.setVectorPercent(0);

      if (requestIntake) {
        nextSystemState = IntakeState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = IntakeState.SPIT;
      }
    } else if (systemState == IntakeState.INTAKE) {
      io.setIntakePercent(INTAKE_SPEED);
      // io.setVectorVelocity(3000);
      io.setVectorPercent(0.5);

      if (!requestIntake) {
        nextSystemState = IntakeState.IDLE;
      }
    } else if (systemState == IntakeState.SPIT) {
      io.setIntakePercent(SPIT_SPEED);
      io.setVectorVelocity(-2000);

      if (!requestSpit) {
        nextSystemState = IntakeState.IDLE;
      }
    }

    if (systemState != nextSystemState) {
      stateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
  }

  public boolean hasPiece() {
    return inputs.beamBreakTriggered;
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
