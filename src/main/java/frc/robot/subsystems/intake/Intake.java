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
    Logger.recordOutput("Intake/hasPieceBeamBreak", hasPiece());
    // Logger.recordOutput("Intake/hasPieceCurrent", hasPieceCurrent());

    // Handle statemachine logic
    IntakeState nextSystemState = systemState;
    if (systemState == IntakeState.IDLE) {
      io.setIntakePercent(0);
      io.setVectorPercent(0);

      if (requestIntake && !RobotContainer.superstructure.hasPiece()) {
        nextSystemState = IntakeState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = IntakeState.SPIT;
      }
    } else if (systemState == IntakeState.INTAKE) {
      if (RobotContainer.superstructure.getSystemState() == SuperstructureState.INTAKE
          && RobotContainer.superstructure.getPivotAngle().getDegrees() < 0.0) {
        io.setIntakePercent(INTAKE_SPEED);
        io.setVectorPercent(0.5);
      } else {
        io.setIntakePercent(0.0);
        io.setVectorPercent(0.0);
      }

      if (!requestIntake || RobotContainer.superstructure.hasPiece()) {
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

  // public boolean hasPieceCurrent() {
  //   return (stateStartTime > 0.25
  //           && Math.abs(inputs.intakeCurrentAmps) > 65
  //           && Math.abs(inputs.intakeVelocityRPM) < 4500)
  //       && DriverStation.isEnabled() == true;
  // }

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
