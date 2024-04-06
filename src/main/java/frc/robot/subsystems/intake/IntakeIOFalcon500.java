package frc.robot.subsystems.intake;

import static frc.robot.constants.Constants.Intake.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commons.LoggedTunableNumber;

public class IntakeIOFalcon500 implements IntakeIO {

  /* Hardware */
  private final TalonFX intake = new TalonFX(INTAKE_ID, "dabus");
  private final TalonFX vector = new TalonFX(VECTOR_ID, "dabus");
  // private final AnalogInput beamBreak = new AnalogInput(0);
  private final DigitalInput beamBreak = new DigitalInput(3);

  /* Configurator */
  private final TalonFXConfigurator intakeConfigurator;
  private final TalonFXConfigurator vectorConfigurator;

  /* Configs */
  private final CurrentLimitsConfigs intakeCurrentLimitConfigs;
  private final CurrentLimitsConfigs vectorCurrentLimitConfigs;

  private final Slot0Configs intakeSlot0Configs;
  private final Slot0Configs vectorSlot0Configs;

  private final MotorOutputConfigs intakeOutputConfigs;
  private final MotorOutputConfigs vectorOutputConfigs;

  /* Gains */
  LoggedTunableNumber intakeKs = new LoggedTunableNumber("Intake/kS", 0.0);
  LoggedTunableNumber intakeKv = new LoggedTunableNumber("Intake/kV", 0.0);
  LoggedTunableNumber intakekP = new LoggedTunableNumber("Intake/kP", 0.0);
  LoggedTunableNumber intakekI = new LoggedTunableNumber("Intake/kI", 0.0);
  LoggedTunableNumber intakeKd = new LoggedTunableNumber("Intake/kD", 0.0);

  LoggedTunableNumber vectorKs = new LoggedTunableNumber("Vector/kS", 0.0);
  LoggedTunableNumber vectorKv = new LoggedTunableNumber("Vector/kV", 0.14);
  LoggedTunableNumber vectorKp = new LoggedTunableNumber("Vector/kP", 0.2);
  LoggedTunableNumber vectorKi = new LoggedTunableNumber("Vector/kI", 0.0);
  LoggedTunableNumber vectorKd = new LoggedTunableNumber("Vector/kD", 0.0);

  private double vectorVelocitySetpoint = 0.0;

  /* Status Signals */
  private StatusSignal<Double> vectorVelocity;
  private StatusSignal<Double> supplyVector;

  public IntakeIOFalcon500() {
    /* Instantiate configuator */
    intakeConfigurator = intake.getConfigurator();
    vectorConfigurator = vector.getConfigurator();

    /* Create configs */
    intakeCurrentLimitConfigs = new CurrentLimitsConfigs();
    intakeCurrentLimitConfigs.SupplyCurrentLimit = 40.0;
    intakeCurrentLimitConfigs.SupplyCurrentThreshold = 40.0;
    intakeCurrentLimitConfigs.StatorCurrentLimit = 300.0;
    intakeCurrentLimitConfigs.SupplyTimeThreshold = 1.5;
    intakeCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

    vectorCurrentLimitConfigs = new CurrentLimitsConfigs();
    vectorCurrentLimitConfigs.SupplyCurrentLimit = 100.0;
    vectorCurrentLimitConfigs.SupplyCurrentThreshold = 100.0;
    vectorCurrentLimitConfigs.SupplyTimeThreshold = 1;
    vectorCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

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

    intakeSlot0Configs = new Slot0Configs();
    intakeSlot0Configs.kS = intakeKs.get();
    intakeSlot0Configs.kV = intakeKv.get();
    intakeSlot0Configs.kP = intakekP.get();
    intakeSlot0Configs.kI = intakekI.get();
    intakeSlot0Configs.kD = intakeKd.get();

    vectorSlot0Configs = new Slot0Configs();
    vectorSlot0Configs.kS = vectorKs.get();
    vectorSlot0Configs.kV = vectorKv.get();
    vectorSlot0Configs.kP = vectorKp.get();
    vectorSlot0Configs.kI = vectorKi.get();
    vectorSlot0Configs.kD = vectorKd.get();

    /* Ramp Configs */
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    /* Apply configs */
    intakeConfigurator.apply(intakeCurrentLimitConfigs);
    intakeConfigurator.apply(intakeOutputConfigs);
    intakeConfigurator.apply(intakeSlot0Configs);

    vectorConfigurator.apply(vectorCurrentLimitConfigs);
    vectorConfigurator.apply(vectorOutputConfigs);
    vectorConfigurator.apply(vectorSlot0Configs);
    vectorConfigurator.apply(openLoopRampsConfigs);
    vectorConfigurator.apply(closedLoopRampsConfigs);

    vectorVelocity = vector.getVelocity();
    supplyVector = vector.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, vectorVelocity, supplyVector);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(vectorVelocity, supplyVector);

    inputs.intakeVelocityRPM = intake.getVelocity().getValueAsDouble() * 60;
    inputs.intakeCurrentAmps = intake.getSupplyCurrent().getValue();
    inputs.intakeAppliedVoltage = intake.getMotorVoltage().getValue();
    inputs.intakeTempCelcius = intake.getDeviceTemp().getValue();

    inputs.vectorVelocityRPM = vectorVelocity.getValue() * 60;
    inputs.vectorAppliedVoltage = vector.getSupplyCurrent().getValue();
    inputs.vectorAppliedVoltage = vector.getMotorVoltage().getValue();
    inputs.vectorTempCelcius = vector.getDeviceTemp().getValue();
    inputs.vectorVelocitySetpoint = vectorVelocitySetpoint;
    inputs.vectorSupplyCurrent = supplyVector.getValueAsDouble();

    // inputs.beamBreakTriggered = beamBreak.getVoltage() > 0.8;
    inputs.beamBreakTriggered = !beamBreak.get();
  }

  @Override
  public void setIntakePercent(double percent) {
    intake.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setVectorPercent(double percent) {
    vector.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setIntakeVelocity(double velocityRpm) {
    intake.setControl(new VelocityVoltage(velocityRpm / 60.0));
  }

  @Override
  public void setVectorVelocity(double velocityRpm) {
    vectorVelocitySetpoint = velocityRpm;
    vector.setControl(new VelocityVoltage(velocityRpm / 60.0));
  }

  @Override
  public void setIntakeCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    intakeCurrentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    intakeCurrentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
    intakeCurrentLimitConfigs.StatorCurrentLimit = currentLimit;

    intakeConfigurator.apply(intakeCurrentLimitConfigs);
  }

  @Override
  public void setVectorCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    vectorCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    vectorCurrentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    vectorCurrentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
    vectorCurrentLimitConfigs.StatorCurrentLimit = currentLimit;

    vectorConfigurator.apply(vectorCurrentLimitConfigs);
  }

  @Override
  public void enableIntakeBrakeMode(boolean enable) {
    intakeOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    intakeConfigurator.apply(intakeOutputConfigs);
  }

  @Override
  public void enableVectorBrakeMode(boolean enable) {
    vectorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    vectorConfigurator.apply(vectorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (intakeKs.hasChanged(0)
        || intakeKv.hasChanged(0)
        || intakekP.hasChanged(0)
        || intakekI.hasChanged(0)
        || intakeKd.hasChanged(0)) {

      intakeSlot0Configs.kS = intakeKs.get();
      intakeSlot0Configs.kV = intakeKv.get();
      intakeSlot0Configs.kP = intakekP.get();
      intakeSlot0Configs.kI = intakekI.get();
      intakeSlot0Configs.kD = intakeKd.get();

      intakeConfigurator.apply(intakeSlot0Configs);
    }

    if (vectorKs.hasChanged(0)
        || vectorKv.hasChanged(0)
        || vectorKp.hasChanged(0)
        || vectorKi.hasChanged(0)
        || vectorKd.hasChanged(0)) {

      vectorSlot0Configs.kS = vectorKs.get();
      vectorSlot0Configs.kV = vectorKv.get();
      vectorSlot0Configs.kP = vectorKp.get();
      vectorSlot0Configs.kI = vectorKi.get();
      vectorSlot0Configs.kD = vectorKd.get();

      vectorConfigurator.apply(vectorSlot0Configs);
    }
  }
}
