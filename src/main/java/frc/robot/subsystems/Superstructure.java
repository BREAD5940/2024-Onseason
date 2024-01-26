package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.ElevatorPivotLowLevel;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.elevatorpivot.ElevatorPivotLowLevel.ElevatorPivotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShotParameter;

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
  private boolean requestIdle = false;
  private boolean requestIntake = false;
  private boolean requestSpit = false;
  private boolean requestGoToSpeakerPose = false;
  private boolean requestGoToAmpPose = false;
  private boolean requestGoToClimb = false;
  private boolean requestClimb = false;
  private boolean requestGoToGoTrapPose = false;

  private boolean wantsShoot = false;
  private boolean wantsShootOverDefense = false;
  private boolean hasGamePiece = false;

  private Supplier<ShotParameter> speakerShotSupplier;

  /* Take in io objects and construct subsystems */
  public Superstructure(ElevatorIO elevatorIO, PivotIO pivotIO, SerializerIO serializerIO, ShooterIO shooterIO,
      IntakeIO intakeIO, Supplier<ShotParameter> speakerShotSupplier) {
    elevatorPivotLowLevel = new ElevatorPivotLowLevel(elevatorIO, pivotIO);
    serializer = new Serializer(serializerIO);
    shooter = new Shooter(shooterIO);
    intake = new Intake(intakeIO);
    this.speakerShotSupplier = speakerShotSupplier'
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
      elevatorPivotLowLevel.requestPursueSetpoint(Rotation2d.fromDegrees(0.0), 0.0);

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
      /* The behavior of this state will change depending on whether or not hasPiece is true */
      if (hasGamePiece) {
        serializer.hold();
        shooter.idleSpeed();
        intake.intake();
        elevatorPivotLowLevel.requestPursueSetpoint(Rotation2d.fromDegrees(0.0), 0.0);

        if (!requestIntake) {
          nextSystemState = SuperstructureState.IDLE;
        } else if (serializer.getBeamBreakTriggered()) {
          hasGamePiece = true;
          nextSystemState = SuperstructureState.IDLE;
          requestIntake = false;
        }
      } else {
        serializer.hold();
        shooter.idleSpeed();
        intake.intake();
        elevatorPivotLowLevel.requestPursueSetpoint(Rotation2d.fromDegrees(0.0), 0.0);

        if (!requestIntake) {
          nextSystemState = SuperstructureState.IDLE;
        } else if (intake.getTopBeamBreakTriggered()) {
          nextSystemState = SuperstructureState.IDLE;
          requestIntake = false;
        }
      }
    } else if (systemState == SuperstructureState.SPIT) {
      if (elevatorPivotLowLevel.atElevatorSetpoint(0.0)
          && elevatorPivotLowLevel.atPivotSetpoint(Rotation2d.fromDegrees(0.0))) {
          serializer.spit();
        } else {
          serializer.hold();
      }
      shooter.stop();
      intake.spit();
      elevatorPivotLowLevel.requestPursueSetpoint(Rotation2d.fromDegrees(0.0), 0.0);

      if (!requestSpit) {
        nextSystemState = SuperstructureState.IDLE;
      }
    } else if (systemState == SuperstructureState.SPEAKER_SCORING_POSE) {
      
    }
  }

  
}
