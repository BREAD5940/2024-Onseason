package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.Shooter.*;

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
import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;

public class ShooterIOKrakenX60 implements ShooterIO {

  /* Hardware */
  private final TalonFX left = new TalonFX(SHOOTER_LEFT_ID);
  private final TalonFX right = new TalonFX(SHOOTER_RIGHT_ID);

  /* Configurators */
  private TalonFXConfigurator leftConfigurator;
  private TalonFXConfigurator rightConfigurator;

  private CurrentLimitsConfigs shooterCurrentLimitConfigs;
  private MotorOutputConfigs leftMotorOutputConfigs;
  private MotorOutputConfigs rightMotorOutputConfigs;
  private Slot0Configs rightShooterSlot0Configs;
  private Slot0Configs leftShooterSlot0Configs;

  /* Setpoints for logging */
  private double desiredLeft = 0.0;
  private double desiredRight = 0.0;

  /* Status signals */
  private StatusSignal<Double> velocityRight;
  private StatusSignal<Double> velocityLeft;
  private StatusSignal<Double> positionRight;
  private StatusSignal<Double> positionLeft;
  private StatusSignal<Double> statorLeft;
  private StatusSignal<Double> statorRight;
  private StatusSignal<Double> supplyLeft;
  private StatusSignal<Double> supplyRight;

  /* Gains */
  /*
   * Tune that worked extremely well testing the new shooter rev on both sides:
   * kP: 0.050000
   * kV: 0.011300
   */
  LoggedTunableNumber rightShooterKs = new LoggedTunableNumber("RightShooter/kS", 0.0);
  LoggedTunableNumber rightShooterKa = new LoggedTunableNumber("RightShooter/kA", 0.0);
  LoggedTunableNumber rightShooterKv = new LoggedTunableNumber("RightShooter/kV", 0.131);
  LoggedTunableNumber rightShooterKp = new LoggedTunableNumber("RightShooter/kP", 0.4);
  LoggedTunableNumber rightShooterKi = new LoggedTunableNumber("RightShooter/kI", 0.0);
  LoggedTunableNumber rightShooterKd = new LoggedTunableNumber("RightShooter/kD", 0.0);

  LoggedTunableNumber leftShooterKs = new LoggedTunableNumber("LeftShooter/kS", 0.0);
  LoggedTunableNumber leftShooterKa = new LoggedTunableNumber("LeftShooter/kA", 0.0);
  LoggedTunableNumber leftShooterKv = new LoggedTunableNumber("LeftShooter/kV", 0.137);
  LoggedTunableNumber leftShooterKp = new LoggedTunableNumber("LeftShooter/kP", 0.4);
  LoggedTunableNumber leftShooterKi = new LoggedTunableNumber("LeftShooter/kI", 0.0);
  LoggedTunableNumber leftShooterKd = new LoggedTunableNumber("LeftShooter/kD", 0.0);

  public ShooterIOKrakenX60() {
    /* Instantiate configuators */
    leftConfigurator = left.getConfigurator();
    rightConfigurator = right.getConfigurator();

    /* Create configs */

    // Current Limit configs
    shooterCurrentLimitConfigs = new CurrentLimitsConfigs();
    shooterCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    shooterCurrentLimitConfigs.StatorCurrentLimit = 100.0;
    shooterCurrentLimitConfigs.SupplyCurrentLimit = 100.0;

    // Motor output configs
    leftMotorOutputConfigs = new MotorOutputConfigs();
    leftMotorOutputConfigs.Inverted = SHOOTER_LEFT_INVERSION;
    leftMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    leftMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    rightMotorOutputConfigs = new MotorOutputConfigs();
    rightMotorOutputConfigs.Inverted = SHOOTER_RIGHT_INVERSION;
    rightMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    rightMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    // Slot 0 Configs (Right)
    rightShooterSlot0Configs = new Slot0Configs();
    rightShooterSlot0Configs.kA = rightShooterKa.get();
    rightShooterSlot0Configs.kS = rightShooterKs.get();
    rightShooterSlot0Configs.kV = rightShooterKv.get();
    rightShooterSlot0Configs.kP = rightShooterKp.get();
    rightShooterSlot0Configs.kI = rightShooterKi.get();
    rightShooterSlot0Configs.kD = rightShooterKd.get();

    // Slot 0 Configs (Left)
    leftShooterSlot0Configs = new Slot0Configs();
    leftShooterSlot0Configs.kA = leftShooterKa.get();
    leftShooterSlot0Configs.kS = leftShooterKs.get();
    leftShooterSlot0Configs.kV = leftShooterKv.get();
    leftShooterSlot0Configs.kP = leftShooterKp.get();
    leftShooterSlot0Configs.kI = leftShooterKi.get();
    leftShooterSlot0Configs.kD = leftShooterKd.get();

    /* Ramp Configs */
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    /* Apply Configurations */
    leftConfigurator.apply(shooterCurrentLimitConfigs);
    leftConfigurator.apply(leftMotorOutputConfigs);
    leftConfigurator.apply(leftShooterSlot0Configs);
    leftConfigurator.apply(openLoopRampsConfigs);
    leftConfigurator.apply(closedLoopRampsConfigs);

    rightConfigurator.apply(shooterCurrentLimitConfigs);
    rightConfigurator.apply(rightMotorOutputConfigs);
    rightConfigurator.apply(rightShooterSlot0Configs);
    leftConfigurator.apply(openLoopRampsConfigs);
    rightConfigurator.apply(closedLoopRampsConfigs);

    /* Status Signals */
    velocityLeft = left.getVelocity();
    velocityRight = right.getVelocity();
    positionLeft = left.getPosition();
    positionRight = right.getPosition();
    statorLeft = left.getStatorCurrent();
    statorRight = right.getStatorCurrent();
    supplyLeft = left.getSupplyCurrent();
    supplyRight = right.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        velocityLeft,
        velocityRight,
        positionLeft,
        positionRight,
        statorLeft,
        statorRight,
        supplyLeft,
        supplyRight);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityLeft,
        velocityRight,
        positionLeft,
        positionRight,
        statorLeft,
        statorRight,
        supplyLeft,
        supplyRight);

    // Shooter left
    inputs.shooterLeftPosRad =
        Units.rotationsToRadians(positionLeft.getValue()) / SHOOTER_LEFT_GEAR_RATIO;
    inputs.shooterLeftVelocityRpm = velocityLeft.getValue() * 60.0;
    inputs.shooterLeftAppliedVolts = left.getMotorVoltage().getValue();
    inputs.shooterLeftTempCelcius = left.getDeviceTemp().getValue();
    inputs.shooterLeftSetpointRPM = desiredLeft;

    // Shooter right
    inputs.shooterRightPosRad =
        Units.rotationsToRadians(positionRight.getValue()) / SHOOTER_RIGHT_GEAR_RATIO;
    inputs.shooterRightVelocityRpm = velocityRight.getValue() * 60.0;
    inputs.shooterRightAppliedVolts = right.getMotorVoltage().getValue();
    inputs.shooterRightTempCelcius = right.getDeviceTemp().getValue();
    inputs.shooterRightSetpointRPM = desiredRight;

    inputs.shooterCurrentAmps = new double[] {supplyLeft.getValue(), supplyRight.getValue()};
  }

  @Override
  public void setPercent(double percentLeft, double percentRight) {
    left.setControl(new DutyCycleOut(percentLeft));
    right.setControl(new DutyCycleOut(percentRight));
  }

  @Override
  public void setVelocity(double velocityLeft, double velocityRight) {
    desiredLeft = velocityLeft;
    desiredRight = velocityRight;

    if (velocityLeft > 0.0) {
      left.setControl(new VelocityVoltage(velocityLeft / 60.0).withSlot(0));
    } else {
      left.setControl(new DutyCycleOut(0.0));
    }

    if (velocityRight > 0.0) {
      right.setControl(new VelocityVoltage(velocityRight / 60.0).withSlot(0));
    } else {
      right.setControl(new DutyCycleOut(0.0));
    }
  }

  @Override
  public void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    shooterCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    shooterCurrentLimitConfigs.StatorCurrentLimit = currentLimit;
    shooterCurrentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    shooterCurrentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;

    leftConfigurator.apply(shooterCurrentLimitConfigs);
    rightConfigurator.apply(shooterCurrentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    leftMotorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    rightMotorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    leftConfigurator.apply(leftMotorOutputConfigs);
    rightConfigurator.apply(rightMotorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (leftShooterKs.hasChanged(0)
        || leftShooterKv.hasChanged(0)
        || leftShooterKp.hasChanged(0)
        || leftShooterKi.hasChanged(0)
        || leftShooterKd.hasChanged(0)
        || leftShooterKa.hasChanged(0)) {
      leftShooterSlot0Configs.kS = leftShooterKs.get();
      leftShooterSlot0Configs.kV = leftShooterKv.get();
      leftShooterSlot0Configs.kP = leftShooterKp.get();
      leftShooterSlot0Configs.kI = leftShooterKi.get();
      leftShooterSlot0Configs.kD = leftShooterKd.get();
      leftShooterSlot0Configs.kA = leftShooterKa.get();

      leftConfigurator.apply(leftShooterSlot0Configs);
    }

    if (rightShooterKs.hasChanged(0)
        || rightShooterKv.hasChanged(0)
        || rightShooterKp.hasChanged(0)
        || rightShooterKi.hasChanged(0)
        || rightShooterKd.hasChanged(0)
        || rightShooterKa.hasChanged(0)) {
      rightShooterSlot0Configs.kS = rightShooterKs.get();
      rightShooterSlot0Configs.kV = rightShooterKv.get();
      rightShooterSlot0Configs.kP = rightShooterKp.get();
      rightShooterSlot0Configs.kI = rightShooterKi.get();
      rightShooterSlot0Configs.kD = rightShooterKd.get();
      rightShooterSlot0Configs.kA = rightShooterKa.get();

      rightConfigurator.apply(rightShooterSlot0Configs);
    }
  }
}
