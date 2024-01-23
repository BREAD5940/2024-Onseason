package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.AutoLog;

public interface SerializerIO {

  @AutoLog
  public class SerializerIOInputs {
    public double positionRads = 0.0;
    public double velocityRpm = 0.0;
    public double currentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
    public boolean limitSwitchTriggered = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SerializerIOInputs inputs) {}

  /** Run open loop at the specified percentage. */
  public default void setPercent(double percent) {}

  /** Run closed loop at the specified velocity */
  public default void setVelocity(double velocityRpm) {}

  /** Sets current limit for the motor serializer */
  public default void setCurrentLimit(
      double statorCurrentLimit,
      double currentLimitTriggerThreshold,
      double currentLimitThresholdTime) {}

  /** Enables or disables brake mode for the motor */
  public default void enableBrakeMode(boolean enable) {}

  /** Updates the tunable numbers */
  public default void updateTunableNumbers() {}
}