package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final boolean isComp = false;
  public static final boolean tuningMode = true;

  public static final double FALCON_FREE_SPEED = 6380.0;
  public static final double KRAKEN_FREE_SPEED = 6000.0;

  /* Constants pertaining to the intake */
  public static class Intake {

    /* IDs */
    public static final int INTAKE_ID = isComp ? 16 : 16;
    public static final int VECTOR_ID = isComp ? 17 : 17;

    /* Setpoints and tolerances */
    public static final double INTAKE_SPEED = isComp ? 1.0 : 1.0;
    public static final double SPIT_SPEED = isComp ? -0.4 : -0.4;
    public static final double FEED_SPEED = isComp ? 0.8 : 0.8;

    /* Physical Measurements */
    public static final InvertedValue INTAKE_INVERSION =
        isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;
    public static final InvertedValue VECTOR_INVERSION =
        isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;

    /* Feedback constants */
    public static final double INTAKE_KV = isComp ? 0.0 : 0.0;
    public static final double INTAKE_KP = isComp ? 0.0 : 0.0;
    public static final double INTAKE_KD = isComp ? 0.0 : 0.0;

    public static final double VECTOR_KV = isComp ? 0.0 : 0.0;
    public static final double VECTOR_KP = isComp ? 0.0 : 0.0;
    public static final double VECTOR_KD = isComp ? 0.0 : 0.0;
  }

  /* Constants pertaining to the swerve drive */
  public static class Swerve {

    public static final double SWERVE_COAST_TRESHOLD_MPS = isComp ? 0.05 : 0.05;
    public static final double SWERVE_COAST_TRESHOLD_SEC = isComp ? 5.0 : 5.0;
    public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD =
        isComp ? Units.degreesToRadians(7) : Units.degreesToRadians(7);
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

    /* IDs */
    public static final int ELEVATOR_LEFT_ID = isComp ? 11 : 11;
    public static final int ELEVATOR_RIGHT_ID = isComp ? 12 : 12;

    /* Elevator setpoints speeds and positions */
    public static final double ELEVATOR_HOMING_VOLTAGE = isComp ? -1 : -1;

    public static final double ELEVATOR_IDLE_HEIGHT = isComp ? 0.125 : 0.125;

    public static final double ELEVATOR_INTAKE_HEIGHT = isComp ? 0.125 : 0.125;

    public static final double ELEVATOR_SPIT_HEIGHT = isComp ? 0.2 : 0.2;

    public static final double ELEVATOR_SPEAKER_DEFENSE_HEIGHT = isComp ? 0.3 : 0.3;

    public static final double ELEVATOR_SPEAKER_SHORT_HEIGHT = isComp ? 0.125 : 0.125;

    public static final double ELEVATOR_AMP_HEIGHT = isComp ? 0.48 : 0.48;

    public static final double ELEVATOR_PRE_CLIMB_HEIGHT = isComp ? 0.4 : 0.4;

    public static final double ELEVATOR_CLIMBED_HEIGHT = isComp ? 0.0 : 0.0;

    public static final double ELEVATOR_TRAP_HEIGHT = isComp ? 0.47 : 0.0;

    public static final double ELEVATOR_FENDER_HEIGHT = isComp ? 0.0 : 0.0;

    /* Physical Measurements */
    public static final double ELEVATOR_SPOOL_DIAMETER =
        isComp ? Units.inchesToMeters(1.463) : Units.inchesToMeters(1.463);
    public static final double ELEVATOR_SETPOINT_TOLERANCE_METERS = isComp ? 0.01 : 0.01;
    public static final double ELEVATOR_HOMING_TRESHOLD_SEC = isComp ? 0.25 : 0.25;
    public static final double ELEVATOR_HOMING_TRESHOLD_MPS = isComp ? 0.01 : 0.01;
    public static final double ELEVATOR_GEAR_RATIO =
        isComp
            ? 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0))
            : 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0));
    public static final double ELEVATOR_MAX_SPEED =
        ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
            * Math.PI
            * ELEVATOR_SPOOL_DIAMETER;
    public static final InvertedValue ELEVATOR_LEFT_INVERSION =
        isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;
    public static final double ELEVATOR_MIN_HEIGHT = isComp ? 0.0 : 0.0;
    public static final double ELEVATOR_END_OF_DANGER_ZONE =
        isComp ? 0.26 : 0.26; // Top gun reference?!?!?!
    public static final double ELEVATOR_MAX_HEIGHT =
        isComp ? 0.48495202978632623 : 0.48495202978632623;

    /* Feedback Constants */
    public static final double ELEVATOR_KA = isComp ? 0.0 : 0.0;
    public static final double ELEVATOR_KS = isComp ? 0.0 : 0.3;
    public static final double ELEVATOR_KV = isComp ? 0.0 : 12.0 / ELEVATOR_MAX_SPEED;
    public static final double ELEVATOR_KP = isComp ? 0.0 : 10;
    public static final double ELEVATOR_KD = isComp ? 0.0 : 0.0;
    public static final double ELEVATOR_MM_CRUISE_VELOCITY = isComp ? 0.0 : 3.0;
    public static final double ELEVATOR_MM_ACCEL = isComp ? 0.0 : 3.0;
  }

  /* Constants pertaining to the pivot */
  public static class Pivot {

    /* IDs and Offsets */
    public static final int PIVOT_ID = isComp ? 18 : 18;
    public static final int PIVOT_AZIMUTH_ID = isComp ? 19 : 19;
    public static final double PIVOT_MAGNET_OFFSET =
        isComp ? Units.degreesToRotations(-33.0) : Units.degreesToRotations(-33.0);

    public static final Rotation2d PIVOT_MAX_ANGLE =
        isComp ? Rotation2d.fromDegrees(33) : Rotation2d.fromDegrees(33);
    public static final Rotation2d PIVOT_MIN_SAFE_ANGLE =
        isComp
            ? Rotation2d.fromDegrees(-12.8)
            : Rotation2d.fromDegrees(
                -12.8); // This would be the minimum rotation at the bottom of the elevator's travel
    public static final Rotation2d PIVOT_MIN_ANGLE =
        isComp
            ? Rotation2d.fromDegrees(-56.42578125)
            : Rotation2d.fromDegrees(
                -56.42578125); // This would be the minumum rotation at any point in the elevator's
    // "safe range"

    /* Pivot setpoint angles */
    public static final Rotation2d PIVOT_NEUTRAL_ANGLE =
        isComp ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_IDLE_ANGLE =
        isComp ? Rotation2d.fromDegrees(-20.0) : Rotation2d.fromDegrees(-20.0);

    public static final Rotation2d PIVOT_INTAKE_ANGLE =
        isComp ? Rotation2d.fromDegrees(-20.0) : Rotation2d.fromDegrees(-20.0);

    public static final Rotation2d PIVOT_SPIT_ANGLE =
        isComp ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(0.0);

    public static final Rotation2d PIVOT_AMP_ANGLE =
        isComp ? Rotation2d.fromDegrees(25.0) : Rotation2d.fromDegrees(25.0);

    public static final Rotation2d PIVOT_PRE_CLIMB_ANGLE =
        isComp ? Rotation2d.fromDegrees(5.0) : Rotation2d.fromDegrees(5.0);

    public static final Rotation2d PIVOT_CLIMBED_ANGLE =
        isComp ? Rotation2d.fromDegrees(5.0) : Rotation2d.fromDegrees(5.0);

    public static final Rotation2d PIVOT_TRAP_ANGLE =
        isComp ? Rotation2d.fromDegrees(5.0) : Rotation2d.fromDegrees(5.0);

    public static final Rotation2d PIVOT_FENDER_ANGLE =
        isComp ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(0.0);

    /* Misc. constants */
    public static final double PIVOT_SETPOINT_TOLERANCE_RADS =
        isComp ? Units.degreesToRadians(3.0) : Units.degreesToRadians(3.0);
    public static final InvertedValue PIVOT_INVERSION =
        isComp ? InvertedValue.CounterClockwise_Positive : InvertedValue.CounterClockwise_Positive;
    public static final SensorDirectionValue PIVOT_ENCODER_INVERSION =
        isComp ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.Clockwise_Positive;
    public static final double PIVOT_GEAR_RATIO = isComp ? 118.0556 : 118.0556;
    public static final double PIVOT_MAX_SPEED =
        ((FALCON_FREE_SPEED / 60.0) * (1.0 / PIVOT_GEAR_RATIO));

    /* Feedback constants */
    public static final double PIVOT_KA = isComp ? 0.0 : 0.0;
    public static final double PIVOT_KS = isComp ? 0.0 : 0.3;
    public static final double PIVOT_KV = isComp ? 0.0 : 12.0 / PIVOT_MAX_SPEED;
    public static final double PIVOT_KG = isComp ? 0.0 : 0.0;
    public static final double PIVOT_KP = isComp ? 0.0 : 75;
    public static final double PIVOT_KD = isComp ? 0.0 : 0;
    public static final double PIVOT_MM_CRUISE_VELOCITY = isComp ? 0.0 : 1.0;
    public static final double PIVOT_MM_ACCEL = isComp ? 0.0 : 1.0;
  }

  /* Constants pertaining to the shooter */
  public static class Shooter {

    /* IDs */
    public static final int SHOOTER_LEFT_ID = isComp ? 13 : 13;
    public static final int SHOOTER_RIGHT_ID = isComp ? 14 : 14;

    /* Setpoints and Tolerances */
    public static final double SHOOTER_SETPOINT_TOLERANCE_RPM = isComp ? 50.0 : 50.0;

    public static final double SHOOTER_LEFT_IDLE_RPM = isComp ? 0.0 : 0.0;
    public static final double SHOOTER_RIGHT_IDLE_RPM = isComp ? 0.0 : 0.0;

    public static final double SHOOTER_LEFT_AMP_RPM = isComp ? 3000.0 : 3000.0;
    public static final double SHOOTER_RIGHT_AMP_RPM = isComp ? 3000.0 : 3000.0;

    public static final double SHOOTER_LEFT_TRAP_RPM = isComp ? 0.0 : 0.0;
    public static final double SHOOTER_RIGHT_TRAP_RPM = isComp ? 0.0 : 0.0;

    public static final double SHOOTER_LEFT_FENDER_RPM = isComp ? 2500.0 : 2500.0;
    public static final double SHOOTER_RIGHT_FENDER_RPM = isComp ? 4500.0 : 4500.0;

    /* Physical Measurements */
    public static final double SHOOTER_LEFT_GEAR_RATIO = isComp ? 1.0 : 1.0;
    public static final double SHOOTER_RIGHT_GEAR_RATIO = isComp ? 1.0 : 1.0;
    public static final double SHOOTER_MAX_VELOCITY = isComp ? (6380.0 / 60.0) : (6380.0 / 60.0);
    public static final InvertedValue SHOOTER_RIGHT_INVERSION =
        isComp ? InvertedValue.CounterClockwise_Positive : InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue SHOOTER_LEFT_INVERSION =
        isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;

    /* Feedback Constants */
    public static final double SHOOTER_KS = 0.0;
    public static final double SHOOTER_KV = 12.0 / SHOOTER_MAX_VELOCITY * (3000.0 / 2400.0);
    public static final double SHOOTER_KP = 1.0;
    public static final double SHOOTER_KD = 0.0;
  }

  /* Constants pertaining to the feeder */
  public static class Feeder {

    /* Feedback Constants */
    public static final double FEEDER_KS = 0.0;
    public static final double FEEDER_KV = 0.0;
    public static final double FEEDER_KP = 0.0;
    public static final double FEEDER_KD = 0.0;

    /* IDs */
    public static final int FEEDER_ID = isComp ? 15 : 15;

    /* Setpoints and tolerances */
    public static final double FEEDER_INTAKE_SPEED = isComp ? 0.3 : 0.3;

    public static final double FEEDER_SPIT_SPEED = isComp ? -0.1 : -0.1;

    public static final double FEEDER_SHOOT_SPEED = isComp ? 0.3 : 0.3;

    /* Physical Measurements */
    public static final double FEEDER_GEAR_RATIO = isComp ? 1.0 : 1.0;
    public static final double FEEDER_ROLLER_DIAMETER = isComp ? 1.0 : 1.0;
    public static final InvertedValue FEEDER_INVERSION =
        isComp ? InvertedValue.CounterClockwise_Positive : InvertedValue.CounterClockwise_Positive;
  }
}
