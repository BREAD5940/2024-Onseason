package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleIntakeIO {
  @AutoLog
  public static class ExampleIntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExampleIntakeIOInputs inputs) {}

  /** Run open loop at the specified percentage */ 
  public default void setPercent(double percent) {}
}