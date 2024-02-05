package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.Shooter.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;

public class ShooterIOKrakenX60 implements ShooterIO {

  /* Hardware */
  private final TalonFX left = new TalonFX(SHOOTER_LEFT_ID, "dabus");
  private final TalonFX right = new TalonFX(SHOOTER_RIGHT_ID, "dabus");

  /* Configurators */
  private TalonFXConfigurator leftConfigurator;
  private TalonFXConfigurator rightConfigurator;

  private CurrentLimitsConfigs currentLimitConfigs;
  private MotorOutputConfigs leftMotorOutputConfigs;
  private MotorOutputConfigs rightMotorOutputConfigs;
  private Slot0Configs slot0Configs;

  /* Gains */
  LoggedTunableNumber flywheelkS = new LoggedTunableNumber("Shooter/kS", 0.0);
  LoggedTunableNumber flywheelkV =
      new LoggedTunableNumber("Shooter/kV", (12.0 / SHOOTER_MAX_SPEED));
  LoggedTunableNumber flywheelkP = new LoggedTunableNumber("Shooter/kP", 0.5);
  LoggedTunableNumber flywheelkI = new LoggedTunableNumber("Shooter/kI", 0.0);
  LoggedTunableNumber flywheelkD = new LoggedTunableNumber("Shooter/kD", 0.0);

  public void ShooterIOFalcon500() {
    /* Instantiate configuators */
    leftConfigurator = left.getConfigurator();
    rightConfigurator = right.getConfigurator();

    /* Create configs */
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = 250.0;

    leftMotorOutputConfigs = new MotorOutputConfigs();
    leftMotorOutputConfigs.Inverted = SHOOTER_LEFT_INVERTED_VALUE;
    leftMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    leftMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    rightMotorOutputConfigs = new MotorOutputConfigs();
    rightMotorOutputConfigs.Inverted = SHOOTER_RIGHT_INVERTED_VALUE;
    rightMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    rightMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = flywheelkS.get();
    slot0Configs.kV = flywheelkV.get();
    slot0Configs.kP = flywheelkP.get();
    slot0Configs.kI = flywheelkI.get();
    slot0Configs.kD = flywheelkD.get();

    /* Apply Configurations */
    leftConfigurator.apply(currentLimitConfigs);
    leftConfigurator.apply(leftMotorOutputConfigs);
    leftConfigurator.apply(slot0Configs);

    rightConfigurator.apply(currentLimitConfigs);
    rightConfigurator.apply(rightMotorOutputConfigs);
    rightConfigurator.apply(slot0Configs);

    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeftPosRad =
        Units.rotationsToRadians(left.getPosition().getValue()) / SHOOTER_LEFT_GEAR_RATIO;
    inputs.shooterLeftVelocityRpm = left.getVelocity().getValue() * 60.0;
    inputs.shooterLeftAppliedVolts = left.getMotorVoltage().getValue();

    inputs.shooterRightPosRad =
        Units.rotationsToRadians(right.getPosition().getValue()) / SHOOTER_RIGHT_GEAR_RATIO;
    inputs.shooterRightVelocityRpm = right.getVelocity().getValue() * 60.0;
    inputs.shooterRightAppliedVolts = right.getMotorVoltage().getValue();

    inputs.currentAmps =
        new double[] {
          left.getSupplyCurrent().getValue(), right.getSupplyCurrent().getValue()
        };
  }

  @Override
  public void setFlywheelPercent(double percentLeft, double percentRight) {
    left.setControl(new DutyCycleOut(percentLeft));
    right.setControl(new DutyCycleOut(percentRight));
  }

  @Override
  public void setFlywheelVelocity(double velocityLeft, double velocityRight) {
    if (velocityLeft > 0.0) {
      left.setControl(new VelocityVoltage(velocityLeft));
    } else {
      left.setControl(new DutyCycleOut(0.0));
    }

    if (velocityRight > 0.0) {
      right.setControl(new VelocityVoltage(velocityRight));
    } else {
      right.setControl(new DutyCycleOut(0.0));
    }
  }

  @Override
  public void setFlywheelCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = currentLimit;
    currentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    currentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;

    leftConfigurator.apply(currentLimitConfigs);
    rightConfigurator.apply(currentLimitConfigs);
  }

  @Override
  public void enableFlywheelBrakeMode(boolean enable) {
    leftMotorOutputConfigs.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    rightMotorOutputConfigs.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    leftConfigurator.apply(leftMotorOutputConfigs);
    rightConfigurator.apply(rightMotorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (flywheelkS.hasChanged(0)
        || flywheelkV.hasChanged(0)
        || flywheelkP.hasChanged(0)
        || flywheelkI.hasChanged(0)
        || flywheelkD.hasChanged(0)) {
      slot0Configs.kS = flywheelkS.get();
      slot0Configs.kV = flywheelkV.get();
      slot0Configs.kP = flywheelkP.get();
      slot0Configs.kI = flywheelkI.get();
      slot0Configs.kD = flywheelkD.get();

      leftConfigurator.apply(slot0Configs);
      rightConfigurator.apply(slot0Configs);
    }
  }
}
