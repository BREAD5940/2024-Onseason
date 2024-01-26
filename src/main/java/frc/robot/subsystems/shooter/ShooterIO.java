package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelLeftPosRad = 0.0;
    public double flywheelLeftVelocityRPM = 0.0;
    public double flywheelLeftAppliedVolts = 0.0;
    public double flywheelLeftTempCelcius = 0.0;

    public double flywheelRightPosRadians = 0.0;
    public double flywheelRightVelocityRPM = 0.0;
    public double flywheelRightAppliedVolts = 0.0;
    public double flywheelRightTempCelcius = 0.0;

    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /* Run open loop at the specified percent. */
  public default void setFlywheelPercent(double percentTop, double percentBottom) {}

  /** Run closed loop at the specified velocity. */
  public default void setFlywheelVelocity(double velocityRpmLeft, double velocityRpmRight) {}

  /* Sets current limit for the flywheel motors. */
  public default void setFlywheelCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /* Enables or disables flywheel brake mode. */
  public default void enableFlywheelBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
