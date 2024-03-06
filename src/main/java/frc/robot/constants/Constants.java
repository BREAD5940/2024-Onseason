// package frc.robot.constants;

// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;

// public class Constants {

//   public static final boolean isComp = false;
//   public static final boolean tuningMode = true;

//   public static final double FALCON_FREE_SPEED = 6380.0;
//   public static final double KRAKEN_FREE_SPEED = 6000.0;

//   /* Constants pertaining to the intake */
//   public static class Intake {

//     /* IDs */
//     public static final int INTAKE_ID = isComp ? 16 : 16;
//     public static final int VECTOR_ID = isComp ? 17 : 17;

//     /* Setpoints and tolerances */
//     public static final double INTAKE_SPEED = isComp ? 1.0 : 1.0;
//     public static final double SPIT_SPEED = isComp ? -0.4 : -0.4;
//     public static final double FEED_SPEED = isComp ? 0.8 : 0.8;

//     /* Physical Measurements */
//     public static final InvertedValue INTAKE_INVERSION =
//         isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;
//     public static final InvertedValue VECTOR_INVERSION =
//         isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;

//     /* Feedback constants */
//     public static final double INTAKE_KV = isComp ? 0.0 : 0.0;
//     public static final double INTAKE_KP = isComp ? 0.0 : 0.0;
//     public static final double INTAKE_KD = isComp ? 0.0 : 0.0;

//     public static final double VECTOR_KV = isComp ? 0.0 : 0.0;
//     public static final double VECTOR_KP = isComp ? 0.0 : 0.0;
//     public static final double VECTOR_KD = isComp ? 0.0 : 0.0;
//   }

//   /* Constants pertaining to the swerve drive */
//   public static class Swerve {

//     public static final double SWERVE_COAST_TRESHOLD_MPS = isComp ? 0.05 : 0.05;
//     public static final double SWERVE_COAST_TRESHOLD_SEC = isComp ? 5.0 : 5.0;
//     public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD =
//         isComp ? Units.degreesToRadians(7) : Units.degreesToRadians(7);
//   }

//   /* Constants pertaining to the elevator */
//   public static class Elevator {

//     /* IDs */
//     public static final int ELEVATOR_LEFT_ID = isComp ? 11 : 11;
//     public static final int ELEVATOR_RIGHT_ID = isComp ? 12 : 12;

//     /* Elevator setpoints speeds and positions */
//     public static final double ELEVATOR_HOMING_VOLTAGE = isComp ? -1 : -1;

//     public static final double ELEVATOR_IDLE_HEIGHT = isComp ? 0.125 : 0.125;

//     public static final double ELEVATOR_INTAKE_HEIGHT = isComp ? 0.125 : 0.125;

//     public static final double ELEVATOR_SPIT_HEIGHT = isComp ? 0.2 : 0.2;

//     public static final double ELEVATOR_SPEAKER_DEFENSE_HEIGHT = isComp ? 0.3 : 0.3;

//     public static final double ELEVATOR_SPEAKER_SHORT_HEIGHT = isComp ? 0.125 : 0.125;

//     public static final double ELEVATOR_AMP_HEIGHT = isComp ? 0.48 : 0.48;

//     public static final double ELEVATOR_PRE_CLIMB_HEIGHT = isComp ? 0.4 : 0.4;

//     public static final double ELEVATOR_CLIMBED_HEIGHT = isComp ? 0.0 : 0.0;

//     public static final double ELEVATOR_TRAP_HEIGHT = isComp ? 0.47 : 0.0;

//     public static final double ELEVATOR_FENDER_HEIGHT = isComp ? 0.0 : 0.0;

//     /* Physical Measurements */
//     public static final double ELEVATOR_SPOOL_DIAMETER =
//         isComp ? Units.inchesToMeters(1.463) : Units.inchesToMeters(1.463);
//     public static final double ELEVATOR_SETPOINT_TOLERANCE_METERS = isComp ? 0.01 : 0.01;
//     public static final double ELEVATOR_HOMING_TRESHOLD_SEC = isComp ? 0.25 : 0.25;
//     public static final double ELEVATOR_HOMING_TRESHOLD_MPS = isComp ? 0.01 : 0.01;
//     public static final double ELEVATOR_GEAR_RATIO =
//         isComp
//             ? 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0))
//             : 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0));
//     public static final double ELEVATOR_MAX_SPEED =
//         ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
//             * Math.PI
//             * ELEVATOR_SPOOL_DIAMETER;
//     public static final InvertedValue ELEVATOR_LEFT_INVERSION =
//         isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;
//     public static final double ELEVATOR_MIN_HEIGHT = isComp ? 0.0 : 0.0;
//     public static final double ELEVATOR_END_OF_DANGER_ZONE =
//         isComp ? 0.26 : 0.26; // Top gun reference?!?!?!
//     public static final double ELEVATOR_MAX_HEIGHT =
//         isComp ? 0.48495202978632623 : 0.48495202978632623;

//     /* Feedback Constants */
//     public static final double ELEVATOR_KA = isComp ? 0.0 : 0.0;
//     public static final double ELEVATOR_KS = isComp ? 0.0 : 0.3;
//     public static final double ELEVATOR_KV = isComp ? 0.0 : 12.0 / ELEVATOR_MAX_SPEED;
//     public static final double ELEVATOR_KP = isComp ? 0.0 : 10;
//     public static final double ELEVATOR_KD = isComp ? 0.0 : 0.0;
//     public static final double ELEVATOR_MM_CRUISE_VELOCITY = isComp ? 0.0 : 3.0;
//     public static final double ELEVATOR_MM_ACCEL = isComp ? 0.0 : 3.0;
//   }

//   /* Constants pertaining to the pivot */
//   public static class Pivot {

//     /* IDs and Offsets */
//     public static final int PIVOT_ID = isComp ? 18 : 18;
//     public static final int PIVOT_AZIMUTH_ID = isComp ? 19 : 19;
//     public static final double PIVOT_MAGNET_OFFSET =
//         isComp ? Units.degreesToRotations(-33.0) : Units.degreesToRotations(-33.0);

//     public static final Rotation2d PIVOT_MAX_ANGLE =
//         isComp ? Rotation2d.fromDegrees(33) : Rotation2d.fromDegrees(33);
//     public static final Rotation2d PIVOT_MIN_SAFE_ANGLE =
//         isComp
//             ? Rotation2d.fromDegrees(-12.8)
//             : Rotation2d.fromDegrees(
//                 -12.8); // This would be the minimum rotation at the bottom of the elevator's
// travel
//     public static final Rotation2d PIVOT_MIN_ANGLE =
//         isComp
//             ? Rotation2d.fromDegrees(-56.42578125)
//             : Rotation2d.fromDegrees(
//                 -56.42578125); // This would be the minumum rotation at any point in the
// elevator's
//     // "safe range"

//     /* Pivot setpoint angles */
//     public static final Rotation2d PIVOT_NEUTRAL_ANGLE =
//         isComp ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(0.0);

//     public static final Rotation2d PIVOT_IDLE_ANGLE =
//         isComp ? Rotation2d.fromDegrees(-20.0) : Rotation2d.fromDegrees(-20.0);

//     public static final Rotation2d PIVOT_INTAKE_ANGLE =
//         isComp ? Rotation2d.fromDegrees(-20.0) : Rotation2d.fromDegrees(-20.0);

//     public static final Rotation2d PIVOT_SPIT_ANGLE =
//         isComp ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(0.0);

//     public static final Rotation2d PIVOT_AMP_ANGLE =
//         isComp ? Rotation2d.fromDegrees(25.0) : Rotation2d.fromDegrees(25.0);

//     public static final Rotation2d PIVOT_PRE_CLIMB_ANGLE =
//         isComp ? Rotation2d.fromDegrees(5.0) : Rotation2d.fromDegrees(5.0);

//     public static final Rotation2d PIVOT_CLIMBED_ANGLE =
//         isComp ? Rotation2d.fromDegrees(5.0) : Rotation2d.fromDegrees(5.0);

//     public static final Rotation2d PIVOT_TRAP_ANGLE =
//         isComp ? Rotation2d.fromDegrees(5.0) : Rotation2d.fromDegrees(5.0);

//     public static final Rotation2d PIVOT_FENDER_ANGLE =
//         isComp ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(0.0);

//     /* Misc. constants */
//     public static final double PIVOT_SETPOINT_TOLERANCE_RADS =
//         isComp ? Units.degreesToRadians(3.0) : Units.degreesToRadians(3.0);
//     public static final InvertedValue PIVOT_INVERSION =
//         isComp ? InvertedValue.CounterClockwise_Positive :
// InvertedValue.CounterClockwise_Positive;
//     public static final SensorDirectionValue PIVOT_ENCODER_INVERSION =
//         isComp ? SensorDirectionValue.Clockwise_Positive :
// SensorDirectionValue.Clockwise_Positive;
//     public static final double PIVOT_GEAR_RATIO = isComp ? 118.0556 : 118.0556;
//     public static final double PIVOT_MAX_SPEED =
//         ((FALCON_FREE_SPEED / 60.0) * (1.0 / PIVOT_GEAR_RATIO));

//     /* Feedback constants */
//     public static final double PIVOT_KA = isComp ? 0.0 : 0.0;
//     public static final double PIVOT_KS = isComp ? 0.0 : 0.3;
//     public static final double PIVOT_KV = isComp ? 0.0 : 12.0 / PIVOT_MAX_SPEED;
//     public static final double PIVOT_KG = isComp ? 0.0 : 0.0;
//     public static final double PIVOT_KP = isComp ? 0.0 : 75;
//     public static final double PIVOT_KD = isComp ? 0.0 : 0;
//     public static final double PIVOT_MM_CRUISE_VELOCITY = isComp ? 0.0 : 1.0;
//     public static final double PIVOT_MM_ACCEL = isComp ? 0.0 : 1.0;
//   }

//   /* Constants pertaining to the shooter */
//   public static class Shooter {

//     /* IDs */
//     public static final int SHOOTER_LEFT_ID = isComp ? 13 : 13;
//     public static final int SHOOTER_RIGHT_ID = isComp ? 14 : 14;

//     /* Setpoints and Tolerances */
//     public static final double SHOOTER_SETPOINT_TOLERANCE_RPM = isComp ? 50.0 : 50.0;

//     public static final double SHOOTER_LEFT_IDLE_RPM = isComp ? 0.0 : 0.0;
//     public static final double SHOOTER_RIGHT_IDLE_RPM = isComp ? 0.0 : 0.0;

//     public static final double SHOOTER_LEFT_AMP_RPM = isComp ? 3000.0 : 3000.0;
//     public static final double SHOOTER_RIGHT_AMP_RPM = isComp ? 3000.0 : 3000.0;

//     public static final double SHOOTER_LEFT_TRAP_RPM = isComp ? 0.0 : 0.0;
//     public static final double SHOOTER_RIGHT_TRAP_RPM = isComp ? 0.0 : 0.0;

//     public static final double SHOOTER_LEFT_FENDER_RPM = isComp ? 2500.0 : 2500.0;
//     public static final double SHOOTER_RIGHT_FENDER_RPM = isComp ? 4500.0 : 4500.0;

//     /* Physical Measurements */
//     public static final double SHOOTER_LEFT_GEAR_RATIO = isComp ? 1.0 : 1.0;
//     public static final double SHOOTER_RIGHT_GEAR_RATIO = isComp ? 1.0 : 1.0;
//     public static final double SHOOTER_MAX_VELOCITY = isComp ? (6380.0 / 60.0) : (6380.0 / 60.0);
//     public static final InvertedValue SHOOTER_RIGHT_INVERSION =
//         isComp ? InvertedValue.CounterClockwise_Positive :
// InvertedValue.CounterClockwise_Positive;
//     public static final InvertedValue SHOOTER_LEFT_INVERSION =
//         isComp ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;

//     /* Feedback Constants */
//     public static final double SHOOTER_KS = 0.0;
//     public static final double SHOOTER_KV = 12.0 / SHOOTER_MAX_VELOCITY * (3000.0 / 2400.0);
//     public static final double SHOOTER_KP = 1.0;
//     public static final double SHOOTER_KD = 0.0;
//   }

//   /* Constants pertaining to the feeder */
//   public static class Feeder {

//     /* Feedback Constants */
//     public static final double FEEDER_KS = 0.0;
//     public static final double FEEDER_KV = 0.0;
//     public static final double FEEDER_KP = 0.0;
//     public static final double FEEDER_KD = 0.0;

//     /* IDs */
//     public static final int FEEDER_ID = isComp ? 15 : 15;

//     /* Setpoints and tolerances */
//     public static final double FEEDER_INTAKE_SPEED = isComp ? 0.3 : 0.3;

//     public static final double FEEDER_SPIT_SPEED = isComp ? -0.1 : -0.1;

//     public static final double FEEDER_SHOOT_SPEED = isComp ? 0.3 : 0.3;

//     /* Physical Measurements */
//     public static final double FEEDER_GEAR_RATIO = isComp ? 1.0 : 1.0;
//     public static final double FEEDER_ROLLER_DIAMETER = isComp ? 1.0 : 1.0;
//     public static final InvertedValue FEEDER_INVERSION =
//         isComp ? InvertedValue.CounterClockwise_Positive :
// InvertedValue.CounterClockwise_Positive;
//   }
// }

package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShotParameter;

// By default these constants are the **Beta** constants
public class Constants {

  public static final RobotType robot = RobotType.BETA;

  public static final boolean tuningMode = true;

  public static final double FALCON_FREE_SPEED = 6380.0;
  public static final double KRAKEN_FREE_SPEED = 6000.0;
  public static final double INWARD_SHIFT_AMOUNT = Units.inchesToMeters(8.0);

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
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(7);
      } else {
        SWERVE_COAST_TRESHOLD_MPS = 0.05;
        SWERVE_COAST_TRESHOLD_SEC = 5.0;
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(7);
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

        ELEVATOR_IDLE_HEIGHT = 0.12;

        ELEVATOR_INTAKE_HEIGHT = 0.12;

        ELEVATOR_SPIT_HEIGHT = 0.2;

        ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.3;

        ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.125;

        ELEVATOR_AMP_HEIGHT = 0.48;

        ELEVATOR_PRE_CLIMB_HEIGHT = 0.55 + Units.inchesToMeters(1.5);

        ELEVATOR_CLIMBED_HEIGHT = 0.00;

        ELEVATOR_TRAP_HEIGHT = 0.65;

        ELEVATOR_FENDER_HEIGHT = 0.4; // 0.4

        ELEVATOR_SPOOL_DIAMETER = Units.inchesToMeters(1.463);
        ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
        ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
        ELEVATOR_HOMING_TRESHOLD_MPS = 0.01;
        ELEVATOR_GEAR_RATIO = 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (36.0 / 30.0));
        ELEVATOR_MAX_SPEED =
            ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
                * Math.PI
                * ELEVATOR_SPOOL_DIAMETER;
        ELEVATOR_LEFT_INVERSION = InvertedValue.CounterClockwise_Positive;
        ELEVATOR_MIN_HEIGHT = 0.0;
        ELEVATOR_END_OF_DANGER_ZONE = 0.328643453491348; // Top gun reference?!?!?!
        ELEVATOR_MAX_HEIGHT = 0.65;
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
        ELEVATOR_END_OF_DANGER_ZONE = 0.34461142807517386; // Top gun reference?!?!?!
        ELEVATOR_MAX_HEIGHT = 0.6342116721107851;
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
    public static final double PIVOT_DELTA_ERROR_TOLERANCE;
    public static final InvertedValue PIVOT_INVERSION;
    public static final SensorDirectionValue PIVOT_ENCODER_INVERSION;
    public static final double PIVOT_GEAR_RATIO;
    public static final double PIVOT_MAX_SPEED;

    static {
      if (robot == RobotType.BETA) {
        PIVOT_ID = 18;
        PIVOT_AZIMUTH_ID = 19;
        PIVOT_MAGNET_OFFSET = Units.degreesToRotations(17.22);

        PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(38.0);
        PIVOT_MIN_SAFE_ANGLE =
            Rotation2d.fromDegrees(
                -20.3); // This would be the minimum rotation at the bottom of the elevator's travel
        PIVOT_MIN_ANGLE =
            Rotation2d.fromDegrees(
                -63.5); // This would be the minumum rotation at any point in the elevator's
        // "safe range"

        PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(-28.0);

        PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(-28.0);

        PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(25.0);

        PIVOT_PRE_CLIMB_ANGLE = Rotation2d.fromDegrees(20.0);

        PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(25.0);

        PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(6.9);

        PIVOT_FENDER_ANGLE = Rotation2d.fromDegrees(-50.0); // -50.0

        PIVOT_SETPOINT_TOLERANCE_RADS = Units.degreesToRadians(1.5);
        PIVOT_DELTA_ERROR_TOLERANCE = 0.002;
        PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
        PIVOT_ENCODER_INVERSION = SensorDirectionValue.Clockwise_Positive;
        PIVOT_GEAR_RATIO = (25.0 / 1.0) * (48.0 / 20.0);
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
        PIVOT_DELTA_ERROR_TOLERANCE = 0.002;
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

        SHOOTER_SETPOINT_TOLERANCE_RPM = 30.0;

        SHOOTER_LEFT_IDLE_RPM = 0.0;
        SHOOTER_RIGHT_IDLE_RPM = 0.0;

        SHOOTER_LEFT_AMP_RPM = 1500.0;
        SHOOTER_RIGHT_AMP_RPM = 1500.0;

        SHOOTER_LEFT_TRAP_RPM = 0.0;
        SHOOTER_RIGHT_TRAP_RPM = 0.0;

        SHOOTER_LEFT_FENDER_RPM = 1400.0; // 1800
        SHOOTER_RIGHT_FENDER_RPM = 2000.0; // 1800

        SHOOTER_LEFT_GEAR_RATIO = 1.0;
        SHOOTER_RIGHT_GEAR_RATIO = 1.0;
        SHOOTER_MAX_VELOCITY = (6380.0 / 60.0);
        SHOOTER_RIGHT_INVERSION = InvertedValue.CounterClockwise_Positive;
        SHOOTER_LEFT_INVERSION = InvertedValue.Clockwise_Positive;
      } else {
        SHOOTER_LEFT_ID = 13;
        SHOOTER_RIGHT_ID = 14;

        SHOOTER_SETPOINT_TOLERANCE_RPM = 50.0;

        SHOOTER_LEFT_IDLE_RPM = 3000.0;
        SHOOTER_RIGHT_IDLE_RPM = 3000.0;

        SHOOTER_LEFT_AMP_RPM = 1000.0;
        SHOOTER_RIGHT_AMP_RPM = 1000.0;

        SHOOTER_LEFT_TRAP_RPM = 0.0;
        SHOOTER_RIGHT_TRAP_RPM = 0.0;

        SHOOTER_LEFT_FENDER_RPM = 3000.0; // 1800
        SHOOTER_RIGHT_FENDER_RPM = 1500.0; // 1800

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

        FEEDER_INTAKE_SPEED = 65; // 65

        FEEDER_SPIT_SPEED = -0.4;

        FEEDER_SHOOT_SPEED = 0.8;

        FEEDER_GEAR_RATIO = 1.0;
        FEEDER_ROLLER_DIAMETER = 1.0;
        FEEDER_INVERSION = InvertedValue.CounterClockwise_Positive;
      } else {
        FEEDER_ID = 15;

        FEEDER_INTAKE_SPEED = 0.75;

        FEEDER_SPIT_SPEED = -0.1;

        FEEDER_SHOOT_SPEED = 0.3;

        FEEDER_GEAR_RATIO = 1.0;
        FEEDER_ROLLER_DIAMETER = 1.0;
        FEEDER_INVERSION = InvertedValue.CounterClockwise_Positive;
      }
    }
  }

  /* Constants pertaining to the shots taken in auto */
  public static class AutoShots {

    // Five Note Amp Side Shots
    public static final ShotParameter FIVE_NOTE_AMP_FIRST_SHOT =
        new ShotParameter(-32.5, 2500, 1200, 0.3);
    public static final ShotParameter FIVE_NOTE_AMP_SECOND_SHOT =
        new ShotParameter(-33.5, 2500, 1200, 0.3);
    public static final ShotParameter FIVE_NOTE_AMP_THIRD_SHOT =
        new ShotParameter(-33.5, 1200, 2500, 0.3);
    public static final ShotParameter FIVE_NOTE_AMP_FOURTH_SHOT =
        new ShotParameter(-20.0, 1500, 3000, 0.2);
  }
}
