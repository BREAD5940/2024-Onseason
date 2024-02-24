package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocityRPM = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeAppliedVoltage = 0.0;
    public double intakeTempCelcius = 0.0;

    public double vectorVelocityRPM = 0.0;
    public double vectorCurrentAmps = 0.0;
    public double vectorAppliedVoltage = 0.0;
    public double vectorTempCelcius = 0.0;

    public boolean beamBreakTriggered = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified percent. */
  public default void setIntakePercent(double percent) {}

  /* Run the vectoring motor open loop */
  public default void setVectorPercent(double percent) {}

  /** Run closed loop at the specified velocity on the intake. */
  public default void setIntakeVelocity(double velocityRpm) {
  }
  
  /** Run closed loop at the specified velocity on the vectoring motor */
  public default void setVectorVelocity(double velocityRPM) {}

  /** Sets current limit for the intake motor. */
  public default void setIntakeCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
  }
      
  /** Sets current limit for the vector motor. */
  public default void setVectorCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /** Enables or disables brake mode for the intake motor. */
  public default void enableIntakeBrakeMode(boolean enable) {
  }
  
  /** Enables or disables brake mode for the intake motor. */
  public default void enableVectorBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
