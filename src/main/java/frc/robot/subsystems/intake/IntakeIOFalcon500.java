package frc.robot.subsystems.intake;

import static frc.robot.constants.RobotConstants.Intake.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commons.LoggedTunableNumber;

public class IntakeIOFalcon500 implements IntakeIO {

  /* Hardware */
  private final TalonFX motor = new TalonFX(INTAKE_ID);
  private final TalonFX vector = new TalonFX(VECTOR_ID, "dabus");
  private final DigitalInput beamBreak = new DigitalInput(9);

  /* Configurator */
  private final TalonFXConfigurator intakeConfigurator;
  private final TalonFXConfigurator vectorConfigurator;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitConfigs;
  private final Slot0Configs slot0Configs;
  private final MotorOutputConfigs intakeOutputConfigs;
  private final MotorOutputConfigs vectorOutputConfigs;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.0);
  LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.0);
  LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);

  public IntakeIOFalcon500() {
    /* Instantiate configuator */
    intakeConfigurator = motor.getConfigurator();
    vectorConfigurator = vector.getConfigurator();

    /* Create configs */
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimit = 50.0;
    currentLimitConfigs.SupplyCurrentThreshold = 50.0;
    currentLimitConfigs.SupplyTimeThreshold = 1;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    intakeOutputConfigs = new MotorOutputConfigs();
    intakeOutputConfigs.Inverted = INTAKE_INVERSION;
    intakeOutputConfigs.PeakForwardDutyCycle = 1.0;
    intakeOutputConfigs.PeakReverseDutyCycle = -1.0;
    intakeOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    vectorOutputConfigs = new MotorOutputConfigs();
    vectorOutputConfigs.Inverted = VECTOR_INVERSION;
    vectorOutputConfigs.PeakForwardDutyCycle = 1.0;
    vectorOutputConfigs.PeakReverseDutyCycle = -1.0;
    vectorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    /* Apply configs */
    intakeConfigurator.apply(currentLimitConfigs);
    intakeConfigurator.apply(intakeOutputConfigs);
    intakeConfigurator.apply(slot0Configs);

    vectorConfigurator.apply(vectorOutputConfigs);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.velocityRpm = motor.getVelocity().getValueAsDouble() * 60;
    inputs.currentAmps = motor.getSupplyCurrent().getValue();
    inputs.appliedVolts = motor.getMotorVoltage().getValue();
    inputs.tempCelcius = motor.getDeviceTemp().getValue();
    inputs.beamBreakTriggered = !beamBreak.get();
  }

  @Override
  public void setIntakePercent(double percent) {
    motor.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setVectorPercent(double percent) {
    vector.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setVelocity(double velocityRpm) {
    motor.setControl(new VelocityVoltage(velocityRpm / 60.0));
  }

  @Override
  public void setCurrentLimit(
      double statorCurrentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    currentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
    currentLimitConfigs.StatorCurrentLimit = statorCurrentLimit;

    intakeConfigurator.apply(currentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    intakeOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    intakeConfigurator.apply(intakeOutputConfigs);
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

      intakeConfigurator.apply(slot0Configs);
    }
  }
}
