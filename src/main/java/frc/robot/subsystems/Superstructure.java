package frc.robot.subsystems;

import static frc.robot.constants.Constants.Elevator.*;
import static frc.robot.constants.Constants.Pivot.*;
import static frc.robot.constants.Constants.Shooter.*;
import static frc.robot.constants.Constants.Intake.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.ElevatorPivot;
import frc.robot.subsystems.elevatorpivot.ElevatorPivot.ElevatorPivotState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShotParameter;
import java.util.function.Function;

/* Superstructure class for handling the interaction between all the subsystems minus swerve */
public class Superstructure extends SubsystemBase {

  /* Subsystems */
  private final ElevatorPivot elevatorPivot;
  private final Feeder feeder;

  /* State variables */
  private SuperstructureState systemState = SuperstructureState.STARTING_CONFIG;
  private double stateStartTime = 0.0;

  private boolean requestHome = true;
  private boolean requestIntake = false;
  private boolean requestSpit = false;
  private boolean requestFender = false;
  private boolean requestVisionSpeaker = false;
  private boolean requestAmp = false;
  private boolean requestPreClimb = false;
  private boolean requestClimb = false;
  private boolean requestTrap = false;

  private boolean wantsShoot = false;
  private boolean wantsShootOverDefense = false;

  private boolean shouldShoot = false;

  private final Function<Boolean, ShotParameter> speakerShotFunction;

  /* System States */
  public enum SuperstructureState {
    STARTING_CONFIG,
    HOMING,
    IDLE,
    INTAKE,
    SPIT,
    FENDER,
    VISION_SPEAKER,
    AMP,
    PRE_CLIMB,
    CLIMB,
    TRAP
  }

  /* Take in io objects and construct subsystems */
    public Superstructure(
      ElevatorIO elevatorIO,
      PivotIO pivotIO,
      FeederIO feederIO,
      Function<Boolean, ShotParameter> speakerShotFunction
    ) {
    elevatorPivot = new ElevatorPivot(elevatorIO, pivotIO);
    feeder = new Feeder(feederIO);
    this.speakerShotFunction = speakerShotFunction;
  }

  /* Call onLoop() of subsystems, handle state machine logic, and log */
  @Override
  public void periodic() {
    /* Call onLoop() methods of subsystems */
    elevatorPivot.onLoop();
    feeder.onLoop();

    /* Handle state machine logic */
    SuperstructureState nextSystemState = systemState;
    if (systemState == SuperstructureState.STARTING_CONFIG) {
      feeder.requestIdle();  

      if (requestHome) {
        elevatorPivot.requestHome();
        nextSystemState = SuperstructureState.HOMING;
      }
    } else if (systemState == SuperstructureState.HOMING) {
      feeder.requestIdle();

      if (elevatorPivot.getSystemState() == ElevatorPivotState.IDLE) {
        nextSystemState = SuperstructureState.IDLE;
        requestHome = false;
      }
    } else if (systemState == SuperstructureState.IDLE) {
      feeder.requestIdle();
      elevatorPivot.requestPursueSetpoint(PIVOT_IDLE_ANGLE, ELEVATOR_IDLE_HEIGHT);

      if (requestHome) {
        nextSystemState = SuperstructureState.HOMING;
      } else if (requestIntake && !feeder.hasPiece()) {
        nextSystemState = SuperstructureState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = SuperstructureState.SPIT;
      } else if (requestFender) {
        nextSystemState = SuperstructureState.FENDER;
      } else if (requestVisionSpeaker) {
        nextSystemState = SuperstructureState.VISION_SPEAKER;
      } else if (requestAmp) {
        nextSystemState = SuperstructureState.AMP;
      } else if (requestPreClimb) {
        nextSystemState = SuperstructureState.PRE_CLIMB;
      } else if (requestClimb) {
        nextSystemState = SuperstructureState.CLIMB;
      } else if (requestTrap) {
        nextSystemState = SuperstructureState.TRAP;
      }
    } else if (systemState == SuperstructureState.INTAKE) {
      feeder.requestIntake();
      elevatorPivot.requestIdle();

      if (!requestIntake) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (feeder.hasPiece()) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.SPIT) {
      feeder.requestSpit();
      elevatorPivot.requestPursueSetpoint(PIVOT_SPIT_ANGLE, ELEVATOR_SPIT_HEIGHT);

      if (!requestSpit) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.VISION_SPEAKER) {
      ShotParameter shot = speakerShotFunction.apply(wantsShootOverDefense);
      if (shouldShoot) {
        feeder.requestShoot();
      } else {
        feeder.requestIdle();
      }
      if (wantsShootOverDefense) {
        elevatorPivot.requestPursueSetpoint(
            new Rotation2d(shot.pivotAngleDeg), ELEVATOR_SPEAKER_DEFENSE_HEIGHT);
      } else {
        elevatorPivot.requestPursueSetpoint(
            new Rotation2d(shot.pivotAngleDeg), ELEVATOR_SPEAKER_SHORT_HEIGHT);
      }

      if (wantsShoot && elevatorPivot.atSetpoint() && RobotContainer.shooter.atSetpoint()) {
        shouldShoot = true;
      }

      if (!requestVisionSpeaker) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (!feeder.hasPiece()) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.AMP) {
      elevatorPivot.requestPursueSetpoint(PIVOT_AMP_ANGLE, ELEVATOR_AMP_HEIGHT);
      if (shouldShoot) {
        feeder.requestShoot();
      } else {
        feeder.requestIdle();
      }

      if (wantsShoot && elevatorPivot.atSetpoint() && RobotContainer.shooter.atSetpoint()) {
        shouldShoot = true;
      }

      if (!requestAmp) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (!feeder.hasPiece()) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.PRE_CLIMB) {
      feeder.requestIdle();
      elevatorPivot.requestPursueSetpoint(PIVOT_PRE_CLIMB_ANGLE, ELEVATOR_PRE_CLIMB_HEIGHT);

      if (requestClimb && elevatorPivot.atSetpoint()) {
        nextSystemState = SuperstructureState.CLIMB;
      } else if (!requestClimb) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.CLIMB) {
      elevatorPivot.requestPursueSetpoint(PIVOT_CLIMBED_ANGLE, ELEVATOR_CLIMBED_HEIGHT);
      feeder.requestIdle();

      if (requestTrap && elevatorPivot.atSetpoint()) {
        nextSystemState = SuperstructureState.TRAP;
      } else {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.TRAP) {
      if (shouldShoot) {
        feeder.requestSpit();
      } else {
        feeder.requestIdle();
      }

      if (wantsShoot && elevatorPivot.atSetpoint()) {
        shouldShoot = true;
      }

      if (!requestTrap) {
        nextSystemState = SuperstructureState.IDLE;
      }
    }

    if (systemState != nextSystemState) {
      stateStartTime = BreadUtil.getFPGATimeSeconds();
      systemState = nextSystemState;
    }
  }

  /** User-facing requests */
  public void requestIdle() {
    unsetAllRequests();
  }

  public void requestHome() {
    unsetAllRequests();
    requestHome = true;
  }

  public void requestSpit() {
    unsetAllRequests();
    requestSpit = true;
  }

  public void requestVisionSpeaker(boolean wantsShoot, boolean wantsShootOverDefense) {
    unsetAllRequests();
    requestVisionSpeaker = true;
    this.wantsShoot = wantsShoot;
    this.wantsShootOverDefense = wantsShootOverDefense;
  }

  public void requestAmp(boolean wantsShoot) {
    unsetAllRequests();
    requestAmp = true;
    this.wantsShoot = wantsShoot;
  }

  public void requestPreClimb() {
    unsetAllRequests();
    requestPreClimb = true;
  }

  public void requestClimb() {
    unsetAllRequests();
    requestClimb = true;
  }

  public void requestTrap(boolean wantsShoot) {
    unsetAllRequests();
    requestTrap = true;
    this.wantsShoot = wantsShoot;
  }

  private void unsetAllRequests() {
    requestHome = false;
    requestIntake = false;
    requestSpit = false;
    requestVisionSpeaker = false;
    requestAmp = false;
    requestPreClimb = false;
    requestClimb = false;
    requestTrap = false;
  }
}
