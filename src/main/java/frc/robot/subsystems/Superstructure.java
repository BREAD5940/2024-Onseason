package frc.robot.subsystems;

import static frc.robot.constants.Constants.Elevator.*;
import static frc.robot.constants.Constants.Pivot.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.ElevatorPivot;
import frc.robot.subsystems.elevatorpivot.ElevatorPivot.ElevatorPivotState;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.shooter.ShotParameter;
import org.littletonrobotics.junction.Logger;

/* Superstructure class for handling the interaction between all the subsystems minus swerve */
public class Superstructure extends SubsystemBase {

  /* Subsystems */
  public final ElevatorPivot elevatorPivot;
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
  public Superstructure(ElevatorIO elevatorIO, PivotIO pivotIO, FeederIO feederIO) {
    elevatorPivot = new ElevatorPivot(elevatorIO, pivotIO);
    feeder = new Feeder(feederIO);
  }

  /* Call onLoop() of subsystems, handle state machine logic, and log */
  @Override
  public void periodic() {
    /* Call onLoop() methods of subsystems */
    elevatorPivot.onLoop();
    feeder.onLoop();

    Logger.recordOutput("Superstructure/State", systemState);
    Logger.recordOutput("Superstructure/ElevatorPivotAtSetpoint", elevatorPivot.atSetpoint());
    Logger.recordOutput("Superstructure/ShooterAtSetpoint", RobotContainer.shooter.atSetpoint());
    Logger.recordOutput("Superstructure/WantsShoot", wantsShoot);
    Logger.recordOutput("Superstructure/hasPiece", hasPiece());

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
      elevatorPivot.requestPursueSetpoint(PIVOT_INTAKE_ANGLE, ELEVATOR_INTAKE_HEIGHT);

      if (requestHome) {
        nextSystemState = SuperstructureState.HOMING;
      }
      if (requestSpit) {
        nextSystemState = SuperstructureState.SPIT;
      } else if (requestPreClimb) {
        nextSystemState = SuperstructureState.PRE_CLIMB;
      } else if (!requestIntake) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (feeder.hasPiece()) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.SPIT) {
      elevatorPivot.requestPursueSetpoint(PIVOT_SPIT_ANGLE, ELEVATOR_SPIT_HEIGHT);
      if (elevatorPivot.atSetpoint()) {
        feeder.requestSpit();
      } else {
        feeder.requestIdle();
      }

      if (requestHome) {
        nextSystemState = SuperstructureState.HOMING;
      } else if (!requestSpit) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.FENDER) {
      if (shouldShoot) {
        feeder.requestShoot();
      } else {
        feeder.requestIdle();
      }
      elevatorPivot.requestPursueSetpoint(PIVOT_FENDER_ANGLE, ELEVATOR_FENDER_HEIGHT);
      if (wantsShoot
          && elevatorPivot.atSetpoint()
          && RobotContainer.shooter.atSetpoint()
          && feeder.hasPiece()) {
        shouldShoot = true;
      }

      if (requestHome) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.HOMING;
      } else if (requestSpit) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.SPIT;
      } else if (!requestFender && !shouldShoot) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (!feeder.hasPiece() && shouldShoot) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.VISION_SPEAKER) {

      ShotParameter shot = RobotContainer.visionSupplier.robotToSpeakerShot();

      if (shouldShoot) {
        feeder.requestShoot();
      } else {
        feeder.requestIdle();
      }

      elevatorPivot.requestPursueSetpoint(
          Rotation2d.fromDegrees(shot.pivotAngleDeg), shot.elevatorHeight);

      if (wantsShoot
          && elevatorPivot.atSetpoint()
          && RobotContainer.shooter.atSetpoint()
          && feeder.hasPiece()) {
        shouldShoot = true;
      }

      if (requestHome) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.HOMING;
      } else if (requestSpit) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.SPIT;
      } else if (!requestVisionSpeaker && !shouldShoot) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (!feeder.hasPiece() && shouldShoot) {
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

      if (wantsShoot
          && elevatorPivot.atSetpoint()
          && RobotContainer.shooter.atSetpoint()
          && feeder.hasPiece()) {
        shouldShoot = true;
      }

      if (requestHome) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.HOMING;
      } else if (requestSpit) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.SPIT;
      } else if (!requestAmp && !shouldShoot) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (shouldShoot && !feeder.hasPiece()) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.PRE_CLIMB) {
      feeder.requestIdle();
      elevatorPivot.requestPursueSetpoint(PIVOT_PRE_CLIMB_ANGLE, ELEVATOR_PRE_CLIMB_HEIGHT);

      if (requestClimb && elevatorPivot.atSetpoint()) {
        nextSystemState = SuperstructureState.CLIMB;
      } else if (!requestPreClimb) {
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

  public void requestHome() {
    requestHome = true;
  }

  public void requestSpit(boolean set) {
    requestSpit = true;
  }

  public void requestFender(boolean set, boolean wantsShoot) {
    requestFender = set;
    this.wantsShoot = wantsShoot;
  }

  public void requestVisionSpeaker(boolean set, boolean wantsShoot, boolean wantsShootOverDefense) {
    requestVisionSpeaker = set;
    this.wantsShoot = wantsShoot;
    this.wantsShootOverDefense = wantsShootOverDefense;
  }

  public void requestAmp(boolean set, boolean wantsShoot) {
    requestAmp = set;
    this.wantsShoot = wantsShoot;
  }

  public void requestIntake(boolean set) {
    requestIntake = set;
  }

  public void requestPreClimb() {
    requestPreClimb = true;
  }

  public void requestClimb() {
    requestClimb = true;
  }

  public void requestTrap(boolean wantsShoot) {
    requestTrap = true;
    this.wantsShoot = wantsShoot;
  }

  public boolean atElevatorPivotSetpoint() {
    return elevatorPivot.atSetpoint();
  }

  public SuperstructureState getSystemState() {
    return systemState;
  }

  public boolean hasPiece() {
    return feeder.hasPiece();
  }
}
