package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

  public static final boolean tuningMode = true;

  /* Constants pertaining to the intake */
  public static class Intake {

    /* IDs */
    public static int INTAKE_ID = 16;

    /* Inversions */
    public static InvertedValue INTAKE_INVERSION = InvertedValue.CounterClockwise_Positive;
  }

  /* Constants pertaining to the swerve drive */
  public static class Swerve {

    public static double COAST_TRESHOLD_MPS = 0.05;
    public static double COAST_TRESHOLD_SEC = 5.0;
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

    /* IDs */
    public static int ELEVATOR_LEFT_ID = 0;
    public static int ELEVATOR_RIGHT_ID = 1;

    /* Limits */
    public static double ELEVATOR_MIN_HEIGHT = 0.0;
    public static double ELEVATOR_END_OF_DANGER_ZONE = 0.0; // Top gun reference?!?!?!
    public static double ELEVATOR_MAX_HEIGHT = 0.0;

    /* Elevator setpoints speeds and positions */
    public static double ELEVATOR_HOMING_PERCENT = -0.3;

    public static double ELEVATOR_IDLE_HEIGHT = 0.0;

    public static double ELEVATOR_INTAKE_HEIGHT = 0.0;

    public static double ELEVATOR_SPIT_HEIGHT = 0.0;

    public static double ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.0;

    public static double ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.0;

    public static double ELEVATOR_AMP_HEIGHT = 0.0;

    public static double ELEVATOR_GO_TO_CLIMB_HEIGHT = 0.0;

    public static double ELEVATOR_CLIMBED_HEIGHT = 0.0;

    public static double ELEVATOR_TRAP_HEIGHT = 0.0;

    /* Misc. constants */
    public static double ELEVATOR_SPOOL_DIAMETER = 0.0;
    public static double ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
    public static double ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
    public static double ELEVATOR_HOMING_TRESHOLD_MPS = 0.1;
    public static double ELEVATOR_GEAR_RATIO = 1.0;
    public static double ELEVATOR_MAX_SPEED = 1.0;
    public static InvertedValue ELEVATOR_LEFT_INVERSION = InvertedValue.CounterClockwise_Positive;
    public static InvertedValue ELEVATOR_RIGHT_INVERSION = InvertedValue.Clockwise_Positive;

    public static double ELEVATOR_S2_HEIGHT = 0.0;

    public static double ELEVATOR_S1_KG = 0.0;

    public static double ELEVATOR_S2_KG = 0.0;
  }

  /* Constants pertaining to the pivot */
  public static class Pivot {

    /* IDs and Offsets */
    public static final int PIVOT_ID = 15;
    public static final int PIVOT_AZIMUTH_ID = 15;
    public static final double PIVOT_MAGNET_OFFSET = 1.0;

    public static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d PIVOT_MIN_SAFE_ANGLE =
        Rotation2d.fromDegrees(
            0.0); // This would be the minimum rotation at the bottom of the elevator's travel
    public static final Rotation2d PIVOT_MIN_ANGLE =
        Rotation2d.fromDegrees(
            0.0); // This would be the minumum rotation at any point in the elevator's "safe range"

    /* Pivot setpoint angles */
    public static final Rotation2d PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(45.0);

    public static final Rotation2d PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_GO_TO_CLIMB_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(0.0);

    /* Misc. constants */
    public static final double PIVOT_SETPOINT_TOLERANCE_RADS = 0.01;
    public static final InvertedValue PIVOT_MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue PIVOT_ENCODER_INVERSION = SensorDirectionValue.CounterClockwise_Positive;
    public static final double PIVOT_GEAR_RATIO = 1.0;
    public static final double PIVOT_MAX_SPEED = ((6380.0 / 60.0) * (1.0 / PIVOT_GEAR_RATIO));
  }

  /* Constants pertaining to the shooter */
  public static class Shooter {

    public static final double SHOOTER_SETPOINT_TOLERANCE_RPM = 10.0;

    public static final int SHOOTER_LEFT_ID = 0;
    public static final int SHOOTER_RIGHT_ID = 0;

    public static final double SHOOTER_LEFT_GEAR_RATIO = 1.0;
    public static final double SHOOTER_RIGHT_GEAR_RATIO = 1.0;

    public static final double SHOOTER_MAX_SPEED = (6380.0 / 60.0);

    public static final InvertedValue SHOOTER_LEFT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue SHOOTER_RIGHT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;


  }

  /* Constants pertaining to the serializer */
  public class Serializer {
    
    /* IDs */
    public static final int SERIALIZER_ID = 15;

    /* Inversions */
    public static final InvertedValue SERIALIZER_INVERSION = InvertedValue.CounterClockwise_Positive;
  }
  
}
