package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

  public static final boolean tuningMode = true;

  /* Constants pertaining to the intake */
  public static class Intake {

    /* IDs */
    public static final int INTAKE_ID = 16;

    /* Setpoints and tolerances */
    public static final double INTAKE_SPEED = 0.0;
    public static final double SPIT_SPEED = 0.0;

    /* Physical Measurements */
    public static final InvertedValue INTAKE_INVERSION = InvertedValue.CounterClockwise_Positive;
  }

  /* Constants pertaining to the swerve drive */
  public static class Swerve {

    public static final double SWERVE_COAST_TRESHOLD_MPS = 0.05;
    public static final double SWERVE_COAST_TRESHOLD_SEC = 5.0;
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

    /* IDs */
    public static final int ELEVATOR_LEFT_ID = 0;
    public static final int ELEVATOR_RIGHT_ID = 1;

    /* Limits */
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_END_OF_DANGER_ZONE = 0.0; // Top gun reference?!?!?!
    public static final double ELEVATOR_MAX_HEIGHT = 0.0;

    /* Elevator setpoints speeds and positions */
    public static final double ELEVATOR_HOMING_PERCENT = -0.3;

    public static final double ELEVATOR_IDLE_HEIGHT = 0.0;

    public static final double ELEVATOR_INTAKE_HEIGHT = 0.0;

    public static final double ELEVATOR_SPIT_HEIGHT = 0.0;

    public static final double ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.0;

    public static final double ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.0;

    public static final double ELEVATOR_AMP_HEIGHT = 0.0;

    public static final double ELEVATOR_PRE_CLIMB_HEIGHT = 0.0;

    public static final double ELEVATOR_CLIMBED_HEIGHT = 0.0;

    public static final double ELEVATOR_TRAP_HEIGHT = 0.0;

    /* Misc. constants */
    public static final double ELEVATOR_SPOOL_DIAMETER = 0.0;
    public static final double ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
    public static final double ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
    public static final double ELEVATOR_HOMING_TRESHOLD_MPS = 0.1;
    public static final double ELEVATOR_GEAR_RATIO = 1.0;
    public static final double ELEVATOR_MAX_SPEED = 1.0;
    public static final InvertedValue ELEVATOR_LEFT_INVERSION =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ELEVATOR_RIGHT_INVERSION = InvertedValue.Clockwise_Positive;

    public static final double ELEVATOR_S2_HEIGHT = 0.0;

    public static final double ELEVATOR_S1_KG = 0.0;

    public static final double ELEVATOR_S2_KG = 0.0;
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

    public static final Rotation2d PIVOT_PRE_CLIMB_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(0.0);

    /* Misc. constants */
    public static final double PIVOT_SETPOINT_TOLERANCE_RADS = 0.01;
    public static final InvertedValue PIVOT_INVERSION = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue PIVOT_ENCODER_INVERSION =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double PIVOT_GEAR_RATIO = 1.0;
    public static final double PIVOT_MAX_SPEED = ((6380.0 / 60.0) * (1.0 / PIVOT_GEAR_RATIO));
  }

  /* Constants pertaining to the shooter */
  public static class Shooter {

    /* IDs */
    public static final int SHOOTER_LEFT_ID = 10;
    public static final int SHOOTER_RIGHT_ID = 11;

    /* Setpoints and Tolerances */
    public static final double SHOOTER_SETPOINT_TOLERANCE_RPM = 10.0;

    public static final double SHOOTER_LEFT_IDLE_RPM = 0.0;
    public static final double SHOOTER_RIGHT_IDLE_RPM = 0.0;

    public static final double SHOOTER_LEFT_AMP_RPM = 0.0;
    public static final double SHOOTER_RIGHT_AMP_RPM = 0.0;

    public static final double SHOOTER_LEFT_TRAP_RPM = 0.0;
    public static final double SHOOTER_RIGHT_TRAP_RPM = 0.0;

    public static final double SHOOTER_LEFT_FENDER_RPM = 0.0;
    public static final double SHOOTER_RIGHT_FENDER_RPM = 0.0;

    /* Physical Measurements */
    public static final double SHOOTER_LEFT_GEAR_RATIO = 1.0;
    public static final double SHOOTER_RIGHT_GEAR_RATIO = 1.0;
    public static final double SHOOTER_MAX_VELOCITY = (6380.0 / 60.0);
    public static final InvertedValue SHOOTER_RIGHT_INVERSION =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue SHOOTER_LEFT_INVERSION =
        InvertedValue.CounterClockwise_Positive;
  }

  /* Constants pertaining to the feeder */
  public static class Feeder {

    /* IDs */
    public static final int FEEDER_ID = 9;

    /* Setpoints and tolerances */
    public static final double FEEDER_INTAKE_SPEED = 0.0;

    public static final double FEEDER_SPIT_SPEED = 0.0;

    public static final double FEEDER_SHOOT_SPEED = 0.0;

    /* Physical Measurements */
    public static final double FEEDER_GEAR_RATIO = 1.0;
    public static final double FEEDER_ROLLER_DIAMETER = 1.0;
    public static final InvertedValue FEEDER_INVERSION = InvertedValue.CounterClockwise_Positive;
  }
}
