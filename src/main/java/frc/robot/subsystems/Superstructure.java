package frc.robot.subsystems;

import static frc.robot.constants.Constants.Elevator.*;
import static frc.robot.constants.Constants.Pivot.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;
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
  // For on-the-fly adjustments
  static LoggedTunableNumber angleAddition =
      new LoggedTunableNumber("On-the-fly/angleAddition", 0.0);

  static LoggedTunableNumber angleMultiplication =
      new LoggedTunableNumber("On-the-fly/distanceScaledAddition", 0.0);

  static LoggedTunableNumber ampAngle =
      new LoggedTunableNumber("Amp/AmpAngle", PIVOT_AMP_ANGLE.getDegrees());
  static LoggedTunableNumber ampHeight =
      new LoggedTunableNumber("Amp/ElevatorHeight", ELEVATOR_AMP_HEIGHT);

  /* Subsystems */
  private final ElevatorPivot elevatorPivot;
  private final Feeder feeder;

  /* State variables */
  private SuperstructureState systemState = SuperstructureState.STARTING_CONFIG;
  private double stateStartTime = 0.0;

  private boolean requestHome = false;
  private boolean requestIntake = false;
  private boolean requestSpit = false;
  private boolean requestFender = false;
  private boolean requestVisionSpeaker = false;
  private boolean requestAmp = false;
  private boolean requestNextClimbState = false;
  private boolean requestPrevClimbState = false;
  private boolean requestVisionSpeakerIntake = false;
  private boolean requestPass = false;

  private boolean wantsShoot = false;
  private boolean wantsShootOverDefense = false;
  private boolean shouldShoot = false;
  private boolean overrideVision = false;
  private double overrideElevatorHeight = 0.0;
  private double overridePivotAngle = 0.0;
  private Pose2d overrideRobotPose = null;

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
    TRAP,
    TRAP_SCORED,
    POST_TRAP,
    PASS,
    VISION_SPEAKER_INTAKE
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
    Logger.recordOutput("Superstructure/requestIntake", requestIntake);
    Logger.recordOutput("Superstructure/requestVisionSpeaker", requestVisionSpeaker);
    Logger.recordOutput("Superstructure/shouldShoot", shouldShoot);

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
      } else if (requestNextClimbState) {
        requestNextClimbState = false;
        nextSystemState = SuperstructureState.PRE_CLIMB;
      } else if (requestPass) {
        nextSystemState = SuperstructureState.PASS;
      } else if (requestVisionSpeakerIntake) {
        nextSystemState = SuperstructureState.VISION_SPEAKER_INTAKE;
      }
    } else if (systemState == SuperstructureState.INTAKE) {
      feeder.requestIntake();
      elevatorPivot.requestPursueSetpoint(PIVOT_INTAKE_ANGLE, ELEVATOR_INTAKE_HEIGHT);

      if (requestHome) {
        nextSystemState = SuperstructureState.HOMING;
      }
      if (requestSpit) {
        nextSystemState = SuperstructureState.SPIT;
      } else if (!requestIntake) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (feeder.hasPiece()) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.SPIT) {
      elevatorPivot.requestPursueSetpoint(PIVOT_SPIT_ANGLE, ELEVATOR_SPIT_HEIGHT);
      if (elevatorPivot.atSetpoint()) {
        feeder.requestSpit(false);
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
      // elevatorPivot.requestPursueSetpoint(
      //     Rotation2d.fromDegrees(Robot.pivotAngle.get()), Robot.elevatorHeight.get());
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
      ShotParameter shot;
      if (overrideVision) {
        shot = new ShotParameter(overridePivotAngle, 0.0, 0.0, overrideElevatorHeight);
      } else {
        if (wantsShootOverDefense) {
          shot = RobotContainer.visionSupplier.robotToSpeakerShotSOD();
        } else {
          shot = RobotContainer.visionSupplier.robotToSpeakerShot();
        }
      }

      if (shouldShoot) {
        feeder.requestShoot();
      } else {
        feeder.requestIdle();
      }

      elevatorPivot.requestPursueSetpoint(
          Rotation2d.fromDegrees((shot.pivotAngleDeg + angleAddition.get())), shot.elevatorHeight);

      Logger.recordOutput("Vision/Shot/ElevatorHeight", shot.elevatorHeight);
      Logger.recordOutput("Vision/Shot/PivotAngleDeg", shot.pivotAngleDeg);
      Logger.recordOutput("Vision/Shot/LeftRpm", shot.leftRPM);
      Logger.recordOutput("Vision/Shot/RightRpm", shot.rightRPM);

      if ((wantsShoot)
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
      } else if (requestIntake && !hasPiece()) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.INTAKE;
      } else if (!requestVisionSpeaker && !shouldShoot) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (!feeder.hasPiece() && shouldShoot) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.AMP) {
      elevatorPivot.requestPursueSetpoint(Rotation2d.fromDegrees(ampAngle.get()), ampHeight.get());
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
      feeder.requestPreTrap();
      elevatorPivot.requestPursueSetpoint(PIVOT_PRE_CLIMB_ANGLE, ELEVATOR_PRE_CLIMB_HEIGHT);

      if (requestNextClimbState) {
        nextSystemState = SuperstructureState.CLIMB;
        requestNextClimbState = false;
      } else if (requestPrevClimbState) {
        nextSystemState = SuperstructureState.IDLE;
        requestPrevClimbState = false;
      }
    } else if (systemState == SuperstructureState.CLIMB) {
      elevatorPivot.requestPursueSetpoint(PIVOT_CLIMBED_ANGLE, ELEVATOR_CLIMBED_HEIGHT);
      feeder.requestIdle();

      if (requestNextClimbState && elevatorPivot.atElevatorSetpoint(ELEVATOR_CLIMBED_HEIGHT)) {
        nextSystemState = SuperstructureState.TRAP;
        requestNextClimbState = false;
      } else if (requestPrevClimbState) {
        nextSystemState = SuperstructureState.IDLE;
        requestPrevClimbState = false;
      }
    } else if (systemState == SuperstructureState.TRAP) {
      if (elevatorPivot.getHeight() > 0.4) {
        elevatorPivot.requestPursueSetpoint(PIVOT_TRAP_ANGLE, ELEVATOR_TRAP_HEIGHT);
      } else {
        elevatorPivot.requestPursueSetpoint(PIVOT_CLIMBED_ANGLE, ELEVATOR_TRAP_HEIGHT);
      }
      feeder.requestIdle();

      if (requestNextClimbState) {
        nextSystemState = SuperstructureState.TRAP_SCORED;
        requestNextClimbState = false;
      } else if (requestPrevClimbState) {
        nextSystemState = SuperstructureState.CLIMB;
        requestPrevClimbState = false;
      }
    } else if (systemState == SuperstructureState.TRAP_SCORED) {
      elevatorPivot.requestPursueSetpoint(PIVOT_TRAP_ANGLE, ELEVATOR_TRAP_HEIGHT);
      feeder.requestSpit(true);
      RobotContainer.shooter.requestTrap();

      if (requestNextClimbState && elevatorPivot.atSetpoint()) {
        nextSystemState = SuperstructureState.POST_TRAP;
        requestNextClimbState = false;
      } else if (requestPrevClimbState) {
        nextSystemState = SuperstructureState.TRAP;
        requestPrevClimbState = false;
      }
    } else if (systemState == SuperstructureState.POST_TRAP) {
      elevatorPivot.requestPursueSetpoint(
          PIVOT_TRAP_ANGLE.plus(Rotation2d.fromDegrees(25.0)),
          (ELEVATOR_TRAP_HEIGHT - 0.2) - Units.inchesToMeters(5.0));
      feeder.requestIdle();

      if (requestPrevClimbState) {
        nextSystemState = SuperstructureState.TRAP;
        requestPrevClimbState = false;
      }
    } else if (systemState == SuperstructureState.PASS) {
      elevatorPivot.requestPursueSetpoint(PIVOT_PASS_ANGLE, ELEVATOR_PASS_HEIGHT);
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
      } else if (!requestPass && !shouldShoot) {
        nextSystemState = SuperstructureState.IDLE;
      } else if (shouldShoot && !feeder.hasPiece()) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (requestIntake && !feeder.hasPiece()) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.VISION_SPEAKER_INTAKE) {
      ShotParameter shot = RobotContainer.visionSupplier.robotToSpeakerShot();

      if (!feeder.hasPiece()) {
        feeder.requestIntake();
      } else {
        feeder.requestShoot();
      }

      elevatorPivot.requestPursueSetpoint(
          Rotation2d.fromDegrees((shot.pivotAngleDeg + angleAddition.get())), shot.elevatorHeight);

      if ((wantsShoot)
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
      } else if (!requestVisionSpeakerIntake && !shouldShoot) {
        shouldShoot = false;
        nextSystemState = SuperstructureState.IDLE;
      } else if (!feeder.hasPiece() && shouldShoot) {
        shouldShoot = false;
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

  public void requestNextClimbState() {
    requestNextClimbState = true;
  }

  public void requestPrevClimbState() {
    requestPrevClimbState = true;
  }

  public void requestVisionSpeakerIntake(boolean set, boolean wantsShoot) {
    requestVisionSpeakerIntake = set;
    this.wantsShoot = wantsShoot;
  }

  public void requestSpit(boolean set) {
    requestSpit = set;
  }

  public void requestFender(boolean set, boolean wantsShoot) {
    requestFender = set;
    this.wantsShoot = wantsShoot;
  }

  public void requestVisionSpeaker(boolean set, boolean wantsShoot, boolean wantsShootOverDefense) {
    requestVisionSpeaker = set;
    this.wantsShoot = wantsShoot;
    this.wantsShootOverDefense = wantsShootOverDefense;
    this.overrideVision = false;
  }

  public void requestVisionSpeaker(
      boolean set,
      boolean wantsShoot,
      boolean wantsShootOverDefense,
      double overridePivotAngle,
      double overrideElevatorHeight) {
    requestVisionSpeaker = set;
    this.wantsShoot = wantsShoot;
    this.wantsShootOverDefense = wantsShootOverDefense;
    this.overrideVision = true;
    this.overridePivotAngle = overridePivotAngle;
    this.overrideElevatorHeight = overrideElevatorHeight;
  }

  public void requestAmp(boolean set, boolean wantsShoot) {
    requestAmp = set;
    this.wantsShoot = wantsShoot;
  }

  public void requestIntake(boolean set) {
    requestIntake = set;
  }

  public void requestPass(boolean set, boolean wantsShoot) {
    requestPass = set;
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

  public boolean wantsShootOverDefense() {
    return wantsShootOverDefense;
  }
}
