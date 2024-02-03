package frc.robot.subsystems.intake;

import static frc.robot.constants.Constants.Intake.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOFalcon500 implements IntakeIO {

  /* Hardware */
  private final TalonFX motor = new TalonFX(INTAKE_ID);

  /* Status Signals */
  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;

  /* Configurator */
  private final TalonFXConfigurator configurator;

  /* Configs */
  private CurrentLimitsConfigs currentLimitConfigs;
  private MotorOutputConfigs motorOutputConfigs;

  public IntakeIOFalcon500() {
    configurator = motor.getConfigurator();

    currentLimitConfigs = new CurrentLimitsConfigs();

    currentLimitConfigs.SupplyCurrentLimit = 370.0;
    currentLimitConfigs.SupplyCurrentThreshold = 370.0;
    currentLimitConfigs.SupplyTimeThreshold = 3.0;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.PeakForwardDutyCycle = 1.0;
    motorOutputConfigs.PeakReverseDutyCycle = -1.0;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    configurator.apply(currentLimitConfigs);
    configurator.apply(motorOutputConfigs);

    position = motor.getPosition();
    velocity = motor.getVelocity();

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValue();
    inputs.currentAmps = motor.getSupplyCurrent().getValue();
  }

  @Override
  public void setPercent(double motorSpeed) {
    motor.setControl(new DutyCycleOut(motorSpeed));
    System.out.println(true);
  }

  @Override
  public void setVelocity(double velocityRpm) {
    motor.setControl(new VelocityVoltage(velocityRpm / 60.0));
  }

  @Override
  public void setCurrentLimit(
      double statorCurrentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    //   currentLimitConfigs.StatorCurrentLimitEnable = true;
    //   currentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    //   currentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
    //   currentLimitConfigs.StatorCurrentLimit = statorCurrentLimit;

    //   configurator.apply(currentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    if (enable) {
      motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    } else {
      motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    }
  }

  @Override
  public void updateTunableNumbers() {}
}
