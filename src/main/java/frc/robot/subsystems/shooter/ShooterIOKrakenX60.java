package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.constants.Constants.Shooter.*;

import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;

public class ShooterIOKrakenX60 implements ShooterIO {
  /* Hardware */
  private final TalonFX flywheelLeft = new TalonFX(FLYWHEEL_LEFT_ID, "dabus");
  private final TalonFX flywheelRight = new TalonFX(FLYWHEEL_RIGHT_ID, "dabus");

  /* Status signals */
  private final StatusSignal<Double> flywheelLeftPosition = flywheelLeft.getPosition();
  private final StatusSignal<Double> flywheelLeftVelocity = flywheelLeft.getVelocity();

  private final StatusSignal<Double> flywheelRightPosition = flywheelRight.getPosition();
  private final StatusSignal<Double> flywheelRightVelocity = flywheelRight.getVelocity();

  /* Configurators */
  private TalonFXConfigurator flywheelLeftConfigurator;
  private TalonFXConfigurator flywheelRightConfigurator;

  private CurrentLimitsConfigs flywheelCurrentLimitConfigs;
  private MotorOutputConfigs flywheelLeftMotorOutputConfigs;
  private MotorOutputConfigs flywheelRightMotorOutputConfigs;
  private Slot0Configs flywheelSlot0Configs;

  /* Gains */
  LoggedTunableNumber flywheelkS = new LoggedTunableNumber("Shooter/kS", 0.0);
  LoggedTunableNumber flywheelkV =
      new LoggedTunableNumber("Shooter/kV", (12.0 / (6380.0 / 60.0)) * (4000.0 / 3870.0));
  LoggedTunableNumber flywheelkP = new LoggedTunableNumber("Shooter/kP", 0.5);
  LoggedTunableNumber flywheelkI = new LoggedTunableNumber("Shooter/kI", 0.0);
  LoggedTunableNumber flywheelkD = new LoggedTunableNumber("Shooter/kD", 0.0);


  public void ShooterIOFalcon500() {
    /* Configurators */
    flywheelLeftConfigurator = flywheelLeft.getConfigurator();
    flywheelRightConfigurator = flywheelRight.getConfigurator();

    /* Configure flywheel hardware */
    flywheelCurrentLimitConfigs = new CurrentLimitsConfigs();
    flywheelCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    flywheelCurrentLimitConfigs.StatorCurrentLimit = 250.0;

    flywheelLeftMotorOutputConfigs = new MotorOutputConfigs();
    flywheelLeftMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    flywheelLeftMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    flywheelLeftMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    flywheelLeftMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    flywheelRightMotorOutputConfigs = new MotorOutputConfigs();
    flywheelRightMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelRightMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    flywheelRightMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    flywheelRightMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    flywheelSlot0Configs = new Slot0Configs();
    flywheelSlot0Configs.kS = flywheelkS.get();
    flywheelSlot0Configs.kV = flywheelkV.get();

    flywheelSlot0Configs.kP = flywheelkP.get();
    flywheelSlot0Configs.kI = flywheelkI.get();
    flywheelSlot0Configs.kD = flywheelkD.get();

    /* Apply Configurations */
    flywheelLeftConfigurator.apply(flywheelCurrentLimitConfigs);
    flywheelLeftConfigurator.apply(flywheelLeftMotorOutputConfigs);
    flywheelLeftConfigurator.apply(flywheelSlot0Configs);

    flywheelRightConfigurator.apply(flywheelCurrentLimitConfigs);
    flywheelRightConfigurator.apply(flywheelRightMotorOutputConfigs);
    flywheelRightConfigurator.apply(flywheelSlot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelLeftPosition,
        flywheelLeftVelocity,
        flywheelRightPosition,
        flywheelRightVelocity);
    flywheelLeft.optimizeBusUtilization();
    flywheelRight.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelLeftPosRad =
        Units.rotationsToRadians(flywheelLeftPosition.getValue()) / FLYWHEEL_LEFT_GEAR_RATIO;
    inputs.flywheelLeftVelocityRpm = flywheelLeft.getVelocity().getValue() * 60.0;
    inputs.flywheelLeftAppliedVolts = flywheelLeft.getMotorVoltage().getValue();

    inputs.flywheelRightPosRad = Units.rotationsToRadians(flywheelRightPosition.getValue()) / FLYWHEEL_RIGHT_GEAR_RATIO;
    inputs.flywheelRightVelocityRpm = flywheelRight.getVelocity().getValue() * 60.0;
    inputs.flywheelRightAppliedVolts = flywheelRight.getMotorVoltage().getValue();

    inputs.currentAmps =  new double[] {flywheelLeft.getSupplyCurrent().getValue(), 
        flywheelRight.getSupplyCurrent().getValue() };
  }

  @Override
  public void setFlywheelPercent(double percentLeft, double percentRight) {
    flywheelLeft.setControl(new DutyCycleOut(percentLeft));
    flywheelRight.setControl(new DutyCycleOut(percentRight));
  }

  @Override
  public void setFlywheelVelocity(double velocityLeft, double velocityRight) {
    if (velocityLeft > 0.0 || velocityRight > 0.0) {
      flywheelLeft.setControl(new VelocityVoltage(velocityLeft / 60.0));
      flywheelRight.setControl(new VelocityVoltage(velocityRight / 60.0));
    } else {
      flywheelLeft.setControl(new DutyCycleOut(0.0));
      flywheelRight.setControl(new DutyCycleOut(0.0));
    }
  }

  @Override
  public void setFlywheelCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    flywheelCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    flywheelCurrentLimitConfigs.StatorCurrentLimit = currentLimit;
    flywheelCurrentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    flywheelCurrentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;

    flywheelLeftConfigurator.apply(flywheelCurrentLimitConfigs);
    flywheelRightConfigurator.apply(flywheelCurrentLimitConfigs);
  }

  @Override
  public void enableFlywheelBrakeMode(boolean enable) {
    flywheelLeftMotorOutputConfigs.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    flywheelRightMotorOutputConfigs.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    flywheelLeftConfigurator.apply(flywheelLeftMotorOutputConfigs);
    flywheelRightConfigurator.apply(flywheelRightMotorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (flywheelkS.hasChanged(0)
        || flywheelkV.hasChanged(0)
        || flywheelkP.hasChanged(0)
        || flywheelkI.hasChanged(0)
        || flywheelkD.hasChanged(0)) {
      flywheelSlot0Configs.kS = flywheelkS.get();
      flywheelSlot0Configs.kV = flywheelkV.get();
      flywheelSlot0Configs.kP = flywheelkP.get();
      flywheelSlot0Configs.kI = flywheelkI.get();
      flywheelSlot0Configs.kD = flywheelkD.get();

      flywheelLeftConfigurator.apply(flywheelSlot0Configs);
      flywheelRightConfigurator.apply(flywheelSlot0Configs);
    }

    System.out.println("Configurations Applied!");
  }
}