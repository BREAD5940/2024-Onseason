package frc.robot.subsystems.elevatorpivot;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double velocityTarget = 0.0;
    public double positionTarget = 0.0;
    public double appliedVolts = 0.0;

    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  /* Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /* Run closed loop to the specified position in meters. */
  public default void setHeight(double heightMeters) {}

  /* Run open loop at the specified percentage. */
  public default void setPercent(double percent) {}

  /* Resets elevator to a specified position in meters. */
  public default void resetHeight(double heightMeters) {}

  /* Sets current limit for the elevator motors. */
  public default void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /* Enables or disables brake mode for the elevator motors. */
  public default void enableBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
