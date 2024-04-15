package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public class FeederIOInputs {
    public double posMeters = 0.0;
    public double velocityMps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
    public double currentAmps = 0.0;
    public boolean beamBreakTriggered = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run open loop at the specified percentage. */
  public default void setPercent(double percent) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRpm) {}

  /** Sets current limit for the feeder motor. */
  public default void setCurrentLimit(
      double statorCurrentLimit,
      double currentLimitTriggerThreshold,
      double currentLimitThresholdTime) {}

  /** Enables or disables brake mode for the motor. */
  public default void enableBrakeMode(boolean enable) {}

  /** Enabled or disables the reverse limit for the motor */
  public default void enableReverseLimit(boolean enable) {}

  /** Updates the tunable numbers */
  public default void updateTunableNumbers() {}
}
