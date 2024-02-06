package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterLeftPosRad = 0.0;
    public double shooterLeftVelocityRpm = 0.0;
    public double shooterLeftAppliedVolts = 0.0;
    public double shooterLeftTempCelcius = 0.0;

    public double shooterRightPosRad = 0.0;
    public double shooterRightVelocityRpm = 0.0;
    public double shooterRightAppliedVolts = 0.0;
    public double shooterRightTempCelcius = 0.0;

    public double[] shooterCurrentAmps = new double[] {}; // {left, right}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /* Run the shooter open loop at the specified percent. */
  public default void setPercent(double percentLeft, double percentRight) {}

  /** Run the shooter closed loop at the specified velocity. */
  public default void setVelocity(double velocityRpmLeft, double velocityRpmRight) {
  }

  /* Sets current limit for the flywheel motors. */
  public default void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /* Enables or disables flywheel brake mode. */
  public default void enableBrakeMode(boolean enable) {}
  
  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
