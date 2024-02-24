package frc.robot.subsystems.intake;

import static frc.robot.constants.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;

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
    FEED,
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
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/SystemState", systemState);

    // Handle statemachine logic
    IntakeState nextSystemState = systemState;
    if (systemState == IntakeState.IDLE) {
      io.setIntakePercent(0);
      io.setVectorPercent(0);

      if (requestIntake && !hasPiece) {
        nextSystemState = IntakeState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = IntakeState.SPIT;
      } else if (RobotContainer.superstructure.getSystemState() == SuperstructureState.INTAKE
          && RobotContainer.superstructure.atElevatorPivotSetpoint()) {
        nextSystemState = IntakeState.FEED;
      }
    } else if (systemState == IntakeState.INTAKE) {
      io.setIntakePercent(INTAKE_SPEED);
      io.setVectorPercent(0.5);

      if (!requestIntake) {
        nextSystemState = IntakeState.IDLE;
      } else if (inputs.beamBreakTriggered) {
        hasPiece = true;
        nextSystemState = IntakeState.IDLE;
      }
    } else if (systemState == IntakeState.SPIT) {
      io.setIntakePercent(SPIT_SPEED);
      io.setVectorPercent(-0.5);

      if (!requestSpit) {
        hasPiece = false;
        nextSystemState = IntakeState.IDLE;
      }
    } else if (systemState == IntakeState.FEED) {
      io.setIntakePercent(FEED_SPEED);
      io.setVectorPercent(0.5);

      if (RobotContainer.superstructure.hasPiece()
          || RobotContainer.superstructure.getSystemState() != SuperstructureState.INTAKE) {
        hasPiece = false;
        nextSystemState = IntakeState.IDLE;
      } else if (requestSpit) {
        nextSystemState = IntakeState.SPIT;
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
