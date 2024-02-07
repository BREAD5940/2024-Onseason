package frc.robot.subsystems.feeder;

import static frc.robot.constants.Constants.Feeder.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commons.LoggedTunableNumber;

public class FeederIOFalcon500 implements FeederIO {

  /* Hardware */
  private final TalonFX motor = new TalonFX(FEEDER_ID, "dabus");
  private final DigitalInput beamBreak = new DigitalInput(0);

  /* Configurator */
  private final TalonFXConfigurator configurator;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitConfigs;
  private final Slot0Configs slot0Configs;
  private final MotorOutputConfigs motorOutputConfigs;
  private double lastSetCurrentLimit;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Feeder/kS", 0.0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Feeder/kV", 0.0);
  LoggedTunableNumber kP = new LoggedTunableNumber("Feeder/kP", 0.0);
  LoggedTunableNumber kI = new LoggedTunableNumber("Feeder/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Feeder/kD", 0.0);

  public FeederIOFalcon500() {
    /* Instantiate configurator */
    configurator = motor.getConfigurator();

    /* Create configs */
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimit = 300.0;
    currentLimitConfigs.SupplyCurrentThreshold = 300.0;
    currentLimitConfigs.SupplyTimeThreshold = 1.0;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.PeakForwardDutyCycle = 1.0;
    motorOutputConfigs.PeakReverseDutyCycle = -1.0;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    /* Apply configs */
    configurator.apply(currentLimitConfigs);
    configurator.apply(motorOutputConfigs);
    configurator.apply(slot0Configs);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.posMeters =
        motor.getPosition().getValue() * Math.PI * FEEDER_ROLLER_DIAMETER / FEEDER_GEAR_RATIO;
    inputs.velocityMps =
        motor.getVelocity().getValue() * Math.PI * FEEDER_ROLLER_DIAMETER / FEEDER_GEAR_RATIO;
    inputs.appliedVolts = motor.getMotorVoltage().getValue();
    inputs.tempCelcius = motor.getDeviceTemp().getValue();
    inputs.currentAmps = motor.getStatorCurrent().getValue();
    inputs.beamBreakTriggered = !beamBreak.get();
  }

  @Override
  public void setPercent(double percent) {
    motor.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setVelocity(double velocityMps) {
    if (velocityMps > 0.0) {
      motor.setControl(
          new VelocityVoltage(
              (velocityMps * FEEDER_GEAR_RATIO) / (Math.PI * FEEDER_ROLLER_DIAMETER)));
    } else {
      motor.setControl(new DutyCycleOut(0.0));
    }
  }

  @Override
  public void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    if (currentLimit != lastSetCurrentLimit) {
      currentLimitConfigs.StatorCurrentLimitEnable = true;
      currentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
      currentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
      currentLimitConfigs.StatorCurrentLimit = currentLimit;

      configurator.apply(currentLimitConfigs);

      lastSetCurrentLimit = currentLimit;
    }
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    configurator.apply(motorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {

      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      configurator.apply(slot0Configs);
    }
  }
}
