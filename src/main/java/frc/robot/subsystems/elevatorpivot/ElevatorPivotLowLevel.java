package frc.robot.subsystems.elevatorpivot;

import static frc.robot.constants.Constants.Elevator.*;
import static frc.robot.constants.Constants.Pivot.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commons.BreadUtil;

public class ElevatorPivotLowLevel {

  /* IO objects encapsulating the hardware involved in the pivot and elevator */
  private final ElevatorIO elevatorIO;
  private final PivotIO pivotIO;

  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  /* System variables and parameters */
  private boolean requestIdle = true;
  private boolean requestHome = false;
  private boolean requestPursueSetpoint = false;

  private ElevatorPivotState systemState = ElevatorPivotState.STARTING_CONFIG;
  private Rotation2d desiredPivotAngle = Rotation2d.fromDegrees(0.0);
  private double desiredElevatorHeight = 0.0;

  private double mStateStartTime = 0.0;

  /* System States */
  public enum ElevatorPivotState {
    STARTING_CONFIG,
    NEUTRALIZING_PIVOT,
    HOMING,
    IDLE,
    PURSUING_SETPOINT
  }

  /* Instantiate io interfaces in constructor */
  public ElevatorPivotLowLevel(ElevatorIO elevatorIO, PivotIO pivotIO) {
    this.elevatorIO = elevatorIO;
    this.pivotIO = pivotIO;
  }

  /* Method to be called periodically */
  public void onLoop() {

    /* Update loggable inputs */
    elevatorIO.updateInputs(elevatorInputs);
    pivotIO.updateInputs(pivotInputs);

    /* Constrain the pivot and elevator setpoints based on the positions of each system */
    Rotation2d adjustedDesiredPivotAngle =
        getPivotMinAngleFromElevatorHeight(desiredElevatorHeight);
    double adjustedDesiredElevatorHeight = getElevatorMinHeightFromPivotAngle(desiredPivotAngle);

    /* Handle statemachine logic */
    ElevatorPivotState nextSystemState = systemState;
    if (systemState == ElevatorPivotState.STARTING_CONFIG) {
      elevatorIO.setPercent(0.0);
      pivotIO.setPercent(0.0);

      if (requestHome) {
        nextSystemState = ElevatorPivotState.NEUTRALIZING_PIVOT;
      }
    } else if (systemState == ElevatorPivotState.NEUTRALIZING_PIVOT) {
      elevatorIO.setPercent(0.0);
      pivotIO.setAngle(PIVOT_NEUTRAL_ANGLE);

      if (atPivotSetpoint(PIVOT_NEUTRAL_ANGLE)) {
        nextSystemState = ElevatorPivotState.HOMING;
      }
    } else if (systemState == ElevatorPivotState.HOMING) {
      elevatorIO.setPercent(ELEVATOR_HOMING_PERCENT);
      pivotIO.setAngle(PIVOT_NEUTRAL_ANGLE);

      if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > ELEVATOR_HOMING_TRESHOLD_SEC
          && Math.abs(elevatorInputs.velMetersPerSecond) < ELEVATOR_HOMING_TRESHOLD_MPS) {
        elevatorIO.resetHeight(0.0);
        nextSystemState = ElevatorPivotState.IDLE;
        requestHome = false;
      }
    } else if (systemState == ElevatorPivotState.IDLE) {
      elevatorIO.setPercent(0.0);
      pivotIO.setAngle(PIVOT_NEUTRAL_ANGLE);

      if (requestHome) {
        nextSystemState = ElevatorPivotState.NEUTRALIZING_PIVOT;
      } else if (requestPursueSetpoint) {
        nextSystemState = ElevatorPivotState.PURSUING_SETPOINT;
      }
    } else if (systemState == ElevatorPivotState.PURSUING_SETPOINT) {
      elevatorIO.setHeight(adjustedDesiredElevatorHeight);
      pivotIO.setAngle(adjustedDesiredPivotAngle);

      if (requestHome) {
        nextSystemState = ElevatorPivotState.NEUTRALIZING_PIVOT;
      } else if (requestIdle) {
        nextSystemState = ElevatorPivotState.IDLE;
      }
    }

    if (nextSystemState != systemState) {
      mStateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
  }

  /* Returns whether or not the pivot is at its setpoint */
  private boolean atPivotSetpoint(Rotation2d setpoint) {
    return BreadUtil.atReference(
        pivotInputs.angleRads, setpoint.getRadians(), PIVOT_SETPOINT_TOLERANCE_RADS, true);
  }

  /* Returns whether or not the elevator is at its setpoint */
  private boolean atElevatorSetpoint(double setpoint) {
    return BreadUtil.atReference(
        elevatorInputs.posMeters, setpoint, ELEVATOR_SETPOINT_TOLERANCE_METERS, true);
  }

  /* Requests the elevator/pivot to pursue a setpoint */
  public void requestPursueSetpoint(Rotation2d pivotAngle, double elevatorHeight) {
    requestPursueSetpoint = true;
    requestIdle = false;
    desiredPivotAngle = pivotAngle;
    desiredElevatorHeight = elevatorHeight;
  }

  /* Requests the elevator/pivot to go into idle mode */
  public void requestIdle() {
    requestIdle = true;
    requestPursueSetpoint = false;
  }

  /* Requests the elevator/pivot to home */
  public void requestHome() {
    requestHome = true;
    requestIdle = false;
    requestPursueSetpoint = false;
  }

  /** Helper methods for collision avoidance */

  /* Returns the minimum angle of the pivot given the elevator height  */
  private static Rotation2d getPivotMinAngleFromElevatorHeight(double height) {
    // Clamp the elevator height to be safe
    height = MathUtil.clamp(height, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT);

    // If the elevator is in the "safe range" then return the absolute minimum angle of the pivot
    if (height > ELEVATOR_END_OF_DANGER_ZONE) {
      return PIVOT_MIN_ANGLE;
    }

    // Normalize elevator height
    double normalizedElevatorHeight =
        BreadUtil.numericalMap(height, ELEVATOR_MIN_HEIGHT, ELEVATOR_END_OF_DANGER_ZONE, 0, 1);

    // Get the arccos of the elevator's mapped Height
    double arccosOfNormalizedElevatorHeight = Math.acos(normalizedElevatorHeight);

    // Map the sin of the normalized elevator height to an pivot angle
    double pivotAngle =
        BreadUtil.numericalMap(
            arccosOfNormalizedElevatorHeight,
            0.0,
            Math.PI,
            PIVOT_MIN_ANGLE.getRadians(),
            PIVOT_MIN_SAFE_ANGLE.getRadians());

    return new Rotation2d(pivotAngle);
  }

  /* Returns the minumum height of the elevator given the pivot angle */
  private static double getElevatorMinHeightFromPivotAngle(Rotation2d pivotAngle) {
    // Clamp the pivot angle to be safe
    pivotAngle =
        new Rotation2d(
            MathUtil.clamp(
                pivotAngle.getRadians(),
                PIVOT_MIN_ANGLE.getRadians(),
                PIVOT_MIN_SAFE_ANGLE.getRadians()));

    // If the pivot is in the "safe range" then return the absolute minimum height of the elevator
    if (pivotAngle.getRadians() > PIVOT_MIN_SAFE_ANGLE.getRadians()) {
      return ELEVATOR_MIN_HEIGHT;
    }

    // Map the pivot angle to a value with 0 and PI
    double arccosOfNormalizedElevatorHeight =
        BreadUtil.numericalMap(
            pivotAngle.getRadians(),
            PIVOT_MIN_ANGLE.getRadians(),
            PIVOT_MIN_SAFE_ANGLE.getRadians(),
            0.0,
            Math.PI);

    // Remove the arccos by taking the cos
    double normalizedElevatorHeight = Math.cos(arccosOfNormalizedElevatorHeight);

    // Map the normalized elevator height to a regular elevator height
    double height =
        BreadUtil.numericalMap(
            normalizedElevatorHeight, 0.0, 1.0, ELEVATOR_MIN_HEIGHT, ELEVATOR_END_OF_DANGER_ZONE);

    return height;
  }
}
