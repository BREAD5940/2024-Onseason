package frc.robot.subsystems;

import static frc.robot.constants.Constants.Elevator.*;
import static frc.robot.constants.Constants.Pivot.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.ElevatorPivotLowLevel;
import frc.robot.subsystems.elevatorpivot.ElevatorPivotLowLevel.ElevatorPivotState;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShotParameter;
import java.util.function.Function;

/* Superstructure class for handling the interaction between all the subsystems minus swerve */
public class Superstructure extends SubsystemBase {

  /* Subsystems */
  private final ElevatorPivotLowLevel elevatorPivotLowLevel;
  private final Serializer serializer;
  private final Shooter shooter;
  private final Intake intake;

  /* State variables */
  private SuperstructureState systemState = SuperstructureState.STARTING_CONFIG;
  private double stateStartTime = 0.0;

  private boolean requestHome = true;
  private boolean requestIntake = false;
  private boolean requestSpit = false;
  private boolean requestGoToSpeakerPose = false;
  private boolean requestGoToAmpPose = false;
  private boolean requestGoToClimb = false;
  private boolean requestClimb = false;
  private boolean requestGoToTrapPose = false;

  private boolean wantsShoot = false;
  private boolean shouldShoot = false;
  private boolean wantsShootOverDefense = false;
  private boolean hasGamePiece = false;

  private Timer shootingTimer = new Timer();

  private Function<Boolean, ShotParameter> speakerShotFunction;

  /* Take in io objects and construct subsystems */
  public Superstructure(
      ElevatorIO elevatorIO,
      PivotIO pivotIO,
      SerializerIO serializerIO,
      ShooterIO shooterIO,
      IntakeIO intakeIO,
      Function<Boolean, ShotParameter> speakerShotFunction) {
    elevatorPivotLowLevel = new ElevatorPivotLowLevel(elevatorIO, pivotIO);
    serializer = new Serializer(serializerIO);
    shooter = new Shooter(shooterIO);
    intake = new Intake(intakeIO);
    this.speakerShotFunction = speakerShotFunction;
    shootingTimer.start();
  }

  /* System States */
  public enum SuperstructureState {
    STARTING_CONFIG,
    HOMING,
    IDLE,
    INTAKE,
    SPIT,
    SPEAKER_SCORING_POSE,
    AMP_SCORING_POSE,
    GO_TO_CLIMB,
    CLIMB,
    AIM_IN_TRAP
  }

  /* Call onLoop() of subsystems, handle state machine logic, and log */
  @Override
  public void periodic() {
    /* Call onLoop() methods of subsystems */
    elevatorPivotLowLevel.onLoop();
    shooter.onLoop();
    intake.onLoop();

    /* Handle state machine logic */
    SuperstructureState nextSystemState = systemState;
    if (systemState == SuperstructureState.STARTING_CONFIG) {
      serializer.stop();
      shooter.stop();
      intake.stop();

      if (requestHome) {
        elevatorPivotLowLevel.requestHome();
        nextSystemState = SuperstructureState.HOMING;
      }
    } else if (systemState == SuperstructureState.HOMING) {
      serializer.stop();
      shooter.stop();
      intake.stop();

      if (elevatorPivotLowLevel.getSystemState() == ElevatorPivotState.IDLE) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.IDLE) {
      serializer.stop();
      shooter.idleSpeed();
      intake.stop();
      elevatorPivotLowLevel.requestPursueSetpoint(PIVOT_IDLE_ANGLE, ELEVATOR_IDLE_HEIGHT);

      if (requestHome) {
        nextSystemState = SuperstructureState.HOMING;
      } else if (requestIntake && !hasGamePiece) {
        nextSystemState = SuperstructureState.INTAKE;
      } else if (requestSpit) {
        nextSystemState = SuperstructureState.SPIT;
      } else if (requestGoToSpeakerPose && hasGamePiece) {
        nextSystemState = SuperstructureState.SPEAKER_SCORING_POSE;
      } else if (requestGoToAmpPose && hasGamePiece) {
        nextSystemState = SuperstructureState.AMP_SCORING_POSE;
      } else if (requestGoToClimb) {
        nextSystemState = SuperstructureState.GO_TO_CLIMB;
      }
    } else if (systemState == SuperstructureState.INTAKE) {
      serializer.hold();
      shooter.idleSpeed();
      intake.intake();
      elevatorPivotLowLevel.requestPursueSetpoint(PIVOT_INTAKE_ANGLE, ELEVATOR_INTAKE_HEIGHT);

      if (!requestIntake) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (intake.getTopBeamBreakTriggered()) {
        nextSystemState = SuperstructureState.IDLE;
        requestIntake = false;
      }
    } else if (systemState == SuperstructureState.SPIT) {
      if (elevatorPivotLowLevel.atSetpoint()) {
        serializer.spit();
      } else {
        serializer.hold();
      }
      shooter.stop();
      intake.spit();
      elevatorPivotLowLevel.requestPursueSetpoint(PIVOT_SPIT_ANGLE, ELEVATOR_SPIT_HEIGHT);

      if (!requestSpit) {
        nextSystemState = SuperstructureState.IDLE;
        hasGamePiece = false;
      }
    } else if (systemState == SuperstructureState.SPEAKER_SCORING_POSE) {
      ShotParameter shot = speakerShotFunction.apply(wantsShootOverDefense);
      if (shouldShoot) {
        serializer.index();
      } else {
        serializer.hold();
      }
      shooter.set(shot.leftRPM, shot.rightRPM);
      intake.stop();
      if (wantsShootOverDefense) {
        elevatorPivotLowLevel.requestPursueSetpoint(
            new Rotation2d(shot.pivotAngleDeg), ELEVATOR_SPEAKER_DEFENSE_HEIGHT);
      } else {
        elevatorPivotLowLevel.requestPursueSetpoint(
            new Rotation2d(shot.pivotAngleDeg), ELEVATOR_SPEAKER_SHORT_HEIGHT);
      }

      if (wantsShoot && elevatorPivotLowLevel.atSetpoint()) {
        shouldShoot = true;
        shootingTimer.reset();
      }

      if (!requestGoToSpeakerPose) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (shouldShoot && shootingTimer.get() > 0.25) {
        shouldShoot = false;
        hasGamePiece = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.AMP_SCORING_POSE) {
      if (shouldShoot) {
        serializer.index();
      } else {
        serializer.hold();
      }
      shooter.set(0.0, 0.0);
      intake.stop();
      elevatorPivotLowLevel.requestPursueSetpoint(PIVOT_AMP_ANGLE, ELEVATOR_AMP_HEIGHT);

      if (wantsShoot && elevatorPivotLowLevel.atSetpoint()) {
        shouldShoot = true;
        shootingTimer.reset();
      }

      if (!requestGoToAmpPose) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (shouldShoot && shootingTimer.get() > 0.25) {
        shouldShoot = false;
        hasGamePiece = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.GO_TO_CLIMB) {
      serializer.hold();
      shooter.idleSpeed();
      intake.stop();
      elevatorPivotLowLevel.requestPursueSetpoint(
          PIVOT_GO_TO_CLIMB_ANGLE, ELEVATOR_GO_TO_CLIMB_HEIGHT);

      if (requestClimb && elevatorPivotLowLevel.atSetpoint()) {
        nextSystemState = SuperstructureState.CLIMB;
      } else if (!requestGoToClimb) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.CLIMB) {
      serializer.hold();
      shooter.idleSpeed();
      intake.stop();
      elevatorPivotLowLevel.requestPursueSetpoint(PIVOT_CLIMBED_ANGLE, ELEVATOR_CLIMBED_HEIGHT);

      if (requestGoToTrapPose && elevatorPivotLowLevel.atSetpoint()) {
        nextSystemState = SuperstructureState.AIM_IN_TRAP;
      } else {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.AIM_IN_TRAP) {
      if (shouldShoot) {
        serializer.index();
      } else {
        serializer.hold();
      }
      shooter.set(0.0, 0.0);
      intake.stop();
      elevatorPivotLowLevel.requestPursueSetpoint(PIVOT_TRAP_ANGLE, ELEVATOR_TRAP_HEIGHT);

      if (wantsShoot && elevatorPivotLowLevel.atSetpoint()) {
        shouldShoot = true;
      }

      if (!requestGoToTrapPose) {
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

  public void requestGoToSpeakerPose(boolean wantsShoot, boolean wantsShootOverDefense) {
    unsetAllRequests();
    requestGoToSpeakerPose = true;
    this.wantsShoot = wantsShoot;
    this.wantsShootOverDefense = wantsShootOverDefense;
  }

  public void requestGoToAmpPose(boolean wantsShoot) {
    unsetAllRequests();
    requestGoToAmpPose = true;
    this.wantsShoot = wantsShoot;
  }

  public void requestGoToClimb() {
    unsetAllRequests();
    requestGoToClimb = true;
  }

  public void requestClimb() {
    unsetAllRequests();
    requestClimb = true;
  }

  public void requestGoToGoTrapPose(boolean wantsShoot) {
    unsetAllRequests();
    requestGoToTrapPose = true;
    this.wantsShoot = wantsShoot;
  }

  private void unsetAllRequests() {
    requestHome = false;
    requestIntake = false;
    requestSpit = false;
    requestGoToSpeakerPose = false;
    requestGoToAmpPose = false;
    requestGoToClimb = false;
    requestClimb = false;
    requestGoToTrapPose = false;
  }
}
