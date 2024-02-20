package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// By default these constants are the **Beta** constants
public class RobotConstants {

  public static final RobotType robot = RobotType.BETA;

  public static final boolean tuningMode = true;

  public static final double FALCON_FREE_SPEED = 6380.0;
  public static final double KRAKEN_FREE_SPEED = 6000.0;

  public enum RobotType {
    BETA,
    GAMMA
  }

  /* Constants pertaining to the intake */
  public static class Intake {

    /* IDs */
    public static final int INTAKE_ID;
    public static final int VECTOR_ID;

    /* Setpoints and tolerances */
    public static final double INTAKE_SPEED;
    public static final double SPIT_SPEED;
    public static final double FEED_SPEED;

    /* Physical Measurements */
    public static final InvertedValue INTAKE_INVERSION;
    public static final InvertedValue VECTOR_INVERSION;

    static {
      if (robot == RobotType.BETA) {
        INTAKE_ID = 16;
        VECTOR_ID = 27;

        INTAKE_SPEED = 1.0;
        SPIT_SPEED = -0.4;
        FEED_SPEED = 0.8;

        INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
        VECTOR_INVERSION = InvertedValue.Clockwise_Positive;
      } else {
        INTAKE_ID = 16;
        VECTOR_ID = 27;

        INTAKE_SPEED = 1.0;
        SPIT_SPEED = -0.4;
        FEED_SPEED = 0.8;

        INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
        VECTOR_INVERSION = InvertedValue.Clockwise_Positive;
      }
    }
  }

  /* Constants pertaining to the swerve drive */
  public static class Swerve {

    public static final double SWERVE_COAST_TRESHOLD_MPS;
    public static final double SWERVE_COAST_TRESHOLD_SEC;
    public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD;

    static {
      if (robot == RobotType.BETA) {
        SWERVE_COAST_TRESHOLD_MPS = 0.05;
        SWERVE_COAST_TRESHOLD_SEC = 5.0;
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(2);
      } else {
        SWERVE_COAST_TRESHOLD_MPS = 0.05;
        SWERVE_COAST_TRESHOLD_SEC = 5.0;
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(2);
      }
    }
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

    /* IDs */
    public static final int ELEVATOR_LEFT_ID;
    public static final int ELEVATOR_RIGHT_ID;

    /* Elevator setpoints speeds and positions */
    public static final double ELEVATOR_HOMING_VOLTAGE;

    public static final double ELEVATOR_IDLE_HEIGHT;

    public static final double ELEVATOR_INTAKE_HEIGHT;

    public static final double ELEVATOR_SPIT_HEIGHT;

    public static final double ELEVATOR_SPEAKER_DEFENSE_HEIGHT;

    public static final double ELEVATOR_SPEAKER_SHORT_HEIGHT;

    public static final double ELEVATOR_AMP_HEIGHT;

    public static final double ELEVATOR_PRE_CLIMB_HEIGHT;

    public static final double ELEVATOR_CLIMBED_HEIGHT;

    public static final double ELEVATOR_TRAP_HEIGHT;

    public static final double ELEVATOR_FENDER_HEIGHT;

    /* Physical Measurements */
    public static final double ELEVATOR_SPOOL_DIAMETER;
    public static final double ELEVATOR_SETPOINT_TOLERANCE_METERS;
    public static final double ELEVATOR_HOMING_TRESHOLD_SEC;
    public static final double ELEVATOR_HOMING_TRESHOLD_MPS;
    public static final double ELEVATOR_GEAR_RATIO;
    public static final double ELEVATOR_MAX_SPEED;
    public static final InvertedValue ELEVATOR_LEFT_INVERSION;
    public static final double ELEVATOR_MIN_HEIGHT;
    public static final double ELEVATOR_END_OF_DANGER_ZONE;
    public static final double ELEVATOR_MAX_HEIGHT;

    static {
      if (robot == RobotType.BETA) {

        ELEVATOR_LEFT_ID = 11;
        ELEVATOR_RIGHT_ID = 12;

        ELEVATOR_HOMING_VOLTAGE = -1;

        ELEVATOR_IDLE_HEIGHT = 0.125;

        ELEVATOR_INTAKE_HEIGHT = 0.125;

        ELEVATOR_SPIT_HEIGHT = 0.2;

        ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.3;

        ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.125;

        ELEVATOR_AMP_HEIGHT = 0.48;

        ELEVATOR_PRE_CLIMB_HEIGHT = 0.4;

        ELEVATOR_CLIMBED_HEIGHT = 0.0;

        ELEVATOR_TRAP_HEIGHT = 0.47;

        ELEVATOR_FENDER_HEIGHT = 0.4;

        ELEVATOR_SPOOL_DIAMETER = Units.inchesToMeters(1.463);
        ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
        ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
        ELEVATOR_HOMING_TRESHOLD_MPS = 0.01;
        ELEVATOR_GEAR_RATIO = 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0));
        ELEVATOR_MAX_SPEED =
            ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
                * Math.PI
                * ELEVATOR_SPOOL_DIAMETER;
        ELEVATOR_LEFT_INVERSION = InvertedValue.Clockwise_Positive;
        ELEVATOR_MIN_HEIGHT = 0.0;
        ELEVATOR_END_OF_DANGER_ZONE = 0.26; // Top gun reference?!?!?!
        ELEVATOR_MAX_HEIGHT = 0.48495202978632623;
      } else {

        ELEVATOR_LEFT_ID = 11;
        ELEVATOR_RIGHT_ID = 12;

        ELEVATOR_HOMING_VOLTAGE = -1;

        ELEVATOR_IDLE_HEIGHT = 0.125;

        ELEVATOR_INTAKE_HEIGHT = 0.125;

        ELEVATOR_SPIT_HEIGHT = 0.2;

        ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.3;

        ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.125;

        ELEVATOR_AMP_HEIGHT = 0.48;

        ELEVATOR_PRE_CLIMB_HEIGHT = 0.4;

        ELEVATOR_CLIMBED_HEIGHT = 0.0;

        ELEVATOR_TRAP_HEIGHT = 0.47;

        ELEVATOR_FENDER_HEIGHT = 0.4;

        ELEVATOR_SPOOL_DIAMETER = Units.inchesToMeters(1.463);
        ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
        ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
        ELEVATOR_HOMING_TRESHOLD_MPS = 0.01;
        ELEVATOR_GEAR_RATIO = 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0));
        ELEVATOR_MAX_SPEED =
            ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
                * Math.PI
                * ELEVATOR_SPOOL_DIAMETER;
        ELEVATOR_LEFT_INVERSION = InvertedValue.Clockwise_Positive;
        ELEVATOR_MIN_HEIGHT = 0.0;
        ELEVATOR_END_OF_DANGER_ZONE = 0.26; // Top gun reference?!?!?!
        ELEVATOR_MAX_HEIGHT = 0.48495202978632623;
      }
    }
  }

  /* Constants pertaining to the pivot */
  public static class Pivot {

    /* IDs and Offsets */
    public static final int PIVOT_ID;
    public static final int PIVOT_AZIMUTH_ID;
    public static final double PIVOT_MAGNET_OFFSET;

    public static final Rotation2d PIVOT_MAX_ANGLE;
    public static final Rotation2d
        PIVOT_MIN_SAFE_ANGLE; // This would be the minimum rotation at the bottom of the elevator's
    // travel
    public static final Rotation2d
        PIVOT_MIN_ANGLE; // This would be the minumum rotation at any point in the elevator's
    // "safe range"

    /* Pivot setpoint angles */
    public static final Rotation2d PIVOT_NEUTRAL_ANGLE;

    public static final Rotation2d PIVOT_IDLE_ANGLE;

    public static final Rotation2d PIVOT_INTAKE_ANGLE;

    public static final Rotation2d PIVOT_SPIT_ANGLE;

    public static final Rotation2d PIVOT_AMP_ANGLE;
    public static final Rotation2d PIVOT_PRE_CLIMB_ANGLE;

    public static final Rotation2d PIVOT_CLIMBED_ANGLE;

    public static final Rotation2d PIVOT_TRAP_ANGLE;

    public static final Rotation2d PIVOT_FENDER_ANGLE;

    /* Physical Measurements */
    public static final double PIVOT_SETPOINT_TOLERANCE_RADS;
    public static final InvertedValue PIVOT_INVERSION;
    public static final SensorDirectionValue PIVOT_ENCODER_INVERSION;
    public static final double PIVOT_GEAR_RATIO;
    public static final double PIVOT_MAX_SPEED;

    static {
      if (robot == RobotType.BETA) {
        PIVOT_ID = 18;
        PIVOT_AZIMUTH_ID = 19;
        PIVOT_MAGNET_OFFSET = Units.degreesToRotations(-33.0);

        PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(33);
        PIVOT_MIN_SAFE_ANGLE =
            Rotation2d.fromDegrees(
                -12.8); // This would be the minimum rotation at the bottom of the elevator's travel
        PIVOT_MIN_ANGLE =
            Rotation2d.fromDegrees(
                -56.42578125); // This would be the minumum rotation at any point in the elevator's
        // "safe range"

        PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(-20.0);

        PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(-20.0);

        PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(25.0);

        PIVOT_PRE_CLIMB_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_FENDER_ANGLE = Rotation2d.fromDegrees(-50.0);

        PIVOT_SETPOINT_TOLERANCE_RADS = Units.degreesToRadians(3.0);
        PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
        PIVOT_ENCODER_INVERSION = SensorDirectionValue.Clockwise_Positive;
        PIVOT_GEAR_RATIO = 118.0556;
        PIVOT_MAX_SPEED = ((FALCON_FREE_SPEED / 60.0) * (1.0 / PIVOT_GEAR_RATIO));
      } else {
        PIVOT_ID = 18;
        PIVOT_AZIMUTH_ID = 19;
        PIVOT_MAGNET_OFFSET = Units.degreesToRotations(-33.0);

        PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(33);
        PIVOT_MIN_SAFE_ANGLE =
            Rotation2d.fromDegrees(
                -12.8); // This would be the minimum rotation at the bottom of the elevator's travel
        PIVOT_MIN_ANGLE =
            Rotation2d.fromDegrees(
                -56.42578125); // This would be the minumum rotation at any point in the elevator's
        // "safe range"

        PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(-20.0);

        PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(-20.0);

        PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(25.0);

        PIVOT_PRE_CLIMB_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_FENDER_ANGLE = Rotation2d.fromDegrees(-50.0);

        PIVOT_SETPOINT_TOLERANCE_RADS = Units.degreesToRadians(3.0);
        PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
        PIVOT_ENCODER_INVERSION = SensorDirectionValue.Clockwise_Positive;
        PIVOT_GEAR_RATIO = 118.0556;
        PIVOT_MAX_SPEED = ((FALCON_FREE_SPEED / 60.0) * (1.0 / PIVOT_GEAR_RATIO));
      }
    }
  }

  /* Constants pertaining to the shooter */
  public static class Shooter {

    /* IDs */
    public static final int SHOOTER_LEFT_ID;
    public static final int SHOOTER_RIGHT_ID;

    /* Setpoints and Tolerances */
    public static final double SHOOTER_SETPOINT_TOLERANCE_RPM;

    public static final double SHOOTER_LEFT_IDLE_RPM;
    public static final double SHOOTER_RIGHT_IDLE_RPM;

    public static final double SHOOTER_LEFT_AMP_RPM;
    public static final double SHOOTER_RIGHT_AMP_RPM;

    public static final double SHOOTER_LEFT_TRAP_RPM;
    public static final double SHOOTER_RIGHT_TRAP_RPM;

    public static final double SHOOTER_LEFT_FENDER_RPM;
    public static final double SHOOTER_RIGHT_FENDER_RPM;

    /* Physical Measurements */
    public static final double SHOOTER_LEFT_GEAR_RATIO;
    public static final double SHOOTER_RIGHT_GEAR_RATIO;
    public static final double SHOOTER_MAX_VELOCITY;
    public static final InvertedValue SHOOTER_RIGHT_INVERSION;
    public static final InvertedValue SHOOTER_LEFT_INVERSION;

    static {
      if (robot == RobotType.BETA) {
        SHOOTER_LEFT_ID = 13;
        SHOOTER_RIGHT_ID = 14;

        SHOOTER_SETPOINT_TOLERANCE_RPM = 50.0;

        SHOOTER_LEFT_IDLE_RPM = 0.0;
        SHOOTER_RIGHT_IDLE_RPM = 0.0;

        SHOOTER_LEFT_AMP_RPM = 3000.0;
        SHOOTER_RIGHT_AMP_RPM = 3000.0;

        SHOOTER_LEFT_TRAP_RPM = 0.0;
        SHOOTER_RIGHT_TRAP_RPM = 0.0;

        SHOOTER_LEFT_FENDER_RPM = 1800.0;
        SHOOTER_RIGHT_FENDER_RPM = 1800.0;

        SHOOTER_LEFT_GEAR_RATIO = 1.0;
        SHOOTER_RIGHT_GEAR_RATIO = 1.0;
        SHOOTER_MAX_VELOCITY = (6380.0 / 60.0);
        SHOOTER_RIGHT_INVERSION = InvertedValue.CounterClockwise_Positive;
        SHOOTER_LEFT_INVERSION = InvertedValue.Clockwise_Positive;
      } else {
        SHOOTER_LEFT_ID = 13;
        SHOOTER_RIGHT_ID = 14;

        SHOOTER_SETPOINT_TOLERANCE_RPM = 50.0;

        SHOOTER_LEFT_IDLE_RPM = 0.0;
        SHOOTER_RIGHT_IDLE_RPM = 0.0;

        SHOOTER_LEFT_AMP_RPM = 3000.0;
        SHOOTER_RIGHT_AMP_RPM = 3000.0;

        SHOOTER_LEFT_TRAP_RPM = 0.0;
        SHOOTER_RIGHT_TRAP_RPM = 0.0;

        SHOOTER_LEFT_FENDER_RPM = 1800.0;
        SHOOTER_RIGHT_FENDER_RPM = 1800.0;

        SHOOTER_LEFT_GEAR_RATIO = 1.0;
        SHOOTER_RIGHT_GEAR_RATIO = 1.0;
        SHOOTER_MAX_VELOCITY = (6380.0 / 60.0);
        SHOOTER_RIGHT_INVERSION = InvertedValue.CounterClockwise_Positive;
        SHOOTER_LEFT_INVERSION = InvertedValue.Clockwise_Positive;
      }
    }
  }

  /* Constants pertaining to the feeder */
  public static class Feeder {

    /* IDs */
    public static final int FEEDER_ID;

    /* Setpoints and tolerances */
    public static final double FEEDER_INTAKE_SPEED;

    public static final double FEEDER_SPIT_SPEED;

    public static final double FEEDER_SHOOT_SPEED;

    /* Physical Measurements */
    public static final double FEEDER_GEAR_RATIO;
    public static final double FEEDER_ROLLER_DIAMETER;
    public static final InvertedValue FEEDER_INVERSION;

    static {
      if (robot == RobotType.BETA) {
        FEEDER_ID = 15;

        FEEDER_INTAKE_SPEED = 0.3;

        FEEDER_SPIT_SPEED = -0.1;

        FEEDER_SHOOT_SPEED = 0.3;

        FEEDER_GEAR_RATIO = 1.0;
        FEEDER_ROLLER_DIAMETER = 1.0;
        FEEDER_INVERSION = InvertedValue.CounterClockwise_Positive;
      } else {
        FEEDER_ID = 15;

        FEEDER_INTAKE_SPEED = 0.3;

        FEEDER_SPIT_SPEED = -0.1;

        FEEDER_SHOOT_SPEED = 0.3;

        FEEDER_GEAR_RATIO = 1.0;
        FEEDER_ROLLER_DIAMETER = 1.0;
        FEEDER_INVERSION = InvertedValue.CounterClockwise_Positive;
      }
    }
  }
}
