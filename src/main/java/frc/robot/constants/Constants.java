package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

  public static final boolean tuningMode = true;

  /* Coast pertaining to the swerve drive */
  public static class Swerve {

    public static double COAST_TRESHOLD_MPS = 0.05;
    public static double COAST_TRESHOLD_SEC = 5.0;
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

    public static double ELEVATOR_MIN_HEIGHT = 0.0;
    public static double ELEVATOR_END_OF_DANGER_ZONE = 0.0; // Top gun reference?!?!?!
    public static double ELEVATOR_MAX_HEIGHT = 0.0;

    /* Elevator setpoints speeds and positions */
    public static double ELEVATOR_HOMING_PERCENT = -0.3;

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

    /* Misc. constants */
    public static double PIVOT_SETPOINT_TOLERANCE_RADS = 0.01;
  }
}
