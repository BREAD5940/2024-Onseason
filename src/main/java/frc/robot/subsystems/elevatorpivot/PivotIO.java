package frc.robot.subsystems.elevatorpivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelcius = 0.0;

    public double absolutePosition = 0.0;
    public double position = 0.0;
    public double positionRads = 0.0;
    public double positionReference = 0.0;
    public double velocityReference = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run closed loop to the specified position. */
  public default void setPivotPosition(Rotation2d position) {}

  /** Run open loop at the specified percentage. */
  public default void setPivotPercent(double percent) {}

  /* Sets current limit for the pivot motor. */
  public default void setPivotCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /** Enables or disables brake mode for the pivot motor. */
  public default void enablePivotBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
