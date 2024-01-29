package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

  public static final boolean tuningMode = true;
  public static final String CANBUS_NAME = "dabus";

  /* Coast pertaining to the swerve drive */
  public static class Swerve {

    public static double COAST_TRESHOLD_MPS = 0.05;
    public static double COAST_TRESHOLD_SEC = 5.0;
  }

  public static class Intake {
    public static int INTAKE_ID = 59;
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

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
    public static double ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
    public static double ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
    public static double ELEVATOR_HOMING_TRESHOLD_MPS = 0.1;
  }

  /* Constants pertaining to the pivot */
  public static class Pivot {

    public static Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0.0);
    public static Rotation2d PIVOT_MIN_SAFE_ANGLE =
        Rotation2d.fromDegrees(
            0.0); // This would be the minimum rotation at the bottom of the elevator's travel
    public static Rotation2d PIVOT_MIN_ANGLE =
        Rotation2d.fromDegrees(
            0.0); // This would be the minumum rotation at any point in the elevator's "safe range"

    /* Pivot setpoint angles */
    public static Rotation2d PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(45.0);

    public static Rotation2d PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(0.0);

    public static Rotation2d PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(0.0);

    public static Rotation2d PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(0.0);

    public static Rotation2d PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(0.0);

    public static Rotation2d PIVOT_GO_TO_CLIMB_ANGLE = Rotation2d.fromDegrees(0.0);

    public static Rotation2d PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(0.0);

    public static Rotation2d PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(0.0);

    /* Misc. constants */
    public static double PIVOT_SETPOINT_TOLERANCE_RADS = 0.01;
    public static int PIVOT_ID = 58;
    public static int PIVOT_GEAR_RATIO = 1;
  }

  /* Constants pertaining to the shooter */
  public static class Shooter {

    public static double SHOOTER_SETPOINT_TOLERANCE_RPM = 10.0;
  }
}
