package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRads = 0.0;
    public double velocityRPM = 0.0;
    public double currentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;

    public boolean topBeamBreakTriggered = false;
    public boolean bottomBeamBreakTriggered = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified percent. */
  public default void setPercent(double percent) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRpm) {}

  /** Sets current limit for the intake motor. */
  public default void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /** Enables or disables brake mode for the motor. */
  public default void enableBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
