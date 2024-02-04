package frc.robot.subsystems.elevatorpivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/* Interface encapsulating pivot hardware */
public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double angleDegrees = 0.0;
    public double angleRads = 0.0;
    public double velDegreesPerSecond = 0.0;
    public double currentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelcius = 0.0;
    public double armTargetPosition = 0.0;
    public double armTargetVelocity = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the desired angle of the pivot */
  public default void setAngle(Rotation2d angle) {}

  /** Sets the speed of the pivot to the desired percent output */
  public default void setPercent(double percent) {}

  /** Sets current limit for the pivot motor. */
  public default void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /** Enables or disables the pivot in brake mode */
  public default void enableBrakeMode(boolean enable) {}

  /** Updates tunable numbers */
  public default void updateTunableNumbers() {}
}
