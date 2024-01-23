package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelTopPosRadians = 0.0;
    public double flywheelTopVelocityRPM = 0.0;
    public double flywheelTopAppliedVolts = 0.0;
    public double flywheelTopTempCelcius = 0.0;

    public double flywheelBottomPosRadians = 0.0;
    public double flywheelBottomVelocityRPM = 0.0;
    public double flywheelBottomAppliedVolts = 0.0;
    public double flywheelBottomTempCelcius = 0.0;

    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /* Run open loop at the specified percent. */
  public default void setFlywheelPercent(double percentTop, double percentBottom) {}

  /** Run closed loop at the specified velocity. */
  public default void setFlywheelVelocity(double velocityRpmTop, double velocityRpmBottom) {}

  /* Sets current limit for the flywheel motors. */
  public default void setFlywheelCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /* Enables or disables flywheel brake mode. */
  public default void enableFlywheelBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}