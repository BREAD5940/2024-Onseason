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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure.SuperstructureState;

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
  LoggedTunableNumber rightShooterKv = new LoggedTunableNumber("RightShooter/kV", 0.132);
  LoggedTunableNumber rightShooterKp = new LoggedTunableNumber("RightShooter/kP", 0.4);
  LoggedTunableNumber rightShooterKi = new LoggedTunableNumber("RightShooter/kI", 0.0);
  LoggedTunableNumber rightShooterKd = new LoggedTunableNumber("RightShooter/kD", 0.0);

  LoggedTunableNumber leftShooterKs = new LoggedTunableNumber("LeftShooter/kS", 0.0);
  LoggedTunableNumber leftShooterKa = new LoggedTunableNumber("LeftShooter/kA", 0.0);
  LoggedTunableNumber leftShooterKv = new LoggedTunableNumber("LeftShooter/kV", 0.133);
  LoggedTunableNumber leftShooterKp = new LoggedTunableNumber("LeftShooter/kP", 0.4);
  LoggedTunableNumber leftShooterKi = new LoggedTunableNumber("LeftShooter/kI", 0.0);
  LoggedTunableNumber leftShooterKd = new LoggedTunableNumber("LeftShooter/kD", 0.0);

  /* NetworkTables */
  private final NetworkTableEntry leftShooterFaultEntry;
  private final NetworkTableEntry leftShooterUndervoltageFaultEntry;
  private final NetworkTableEntry leftShooterBootDuringEnableFaultEntry;
  private final NetworkTableEntry leftShooterBridgeBrownoutFaultEntry;
  private final NetworkTableEntry leftShooterDeviceTempFaultEntry;
  private final NetworkTableEntry leftShooterForwardHardLimitFaultEntry;
  private final NetworkTableEntry leftShooterForwardSoftLimitFaultEntry;
  private final NetworkTableEntry leftShooterHardwareFaultEntry;
  private final NetworkTableEntry leftShooterMissingDifferentialFXFaultEntry;
  private final NetworkTableEntry leftShooterOverSupplyVFaultEntry;
  private final NetworkTableEntry leftShooterProcTempFaultEntry;
  private final NetworkTableEntry leftShooterReverseHardLimitFaultEntry;
  private final NetworkTableEntry leftShooterReverseSoftLimitFaultEntry;
  private final NetworkTableEntry leftShooterStatorCurrLimitFaultEntry;
  private final NetworkTableEntry leftShooterSupplyCurrLimitFaultEntry;
  private final NetworkTableEntry leftShooterUnlicensedFeatureInUseFaultEntry;
  private final NetworkTableEntry leftShooterUnstableSupplyVFaultEntry;

  private final NetworkTableEntry rightShooterFaultEntry;
  private final NetworkTableEntry rightShooterUndervoltageFaultEntry;
  private final NetworkTableEntry rightShooterBootDuringEnableFaultEntry;
  private final NetworkTableEntry rightShooterBridgeBrownoutFaultEntry;
  private final NetworkTableEntry rightShooterDeviceTempFaultEntry;
  private final NetworkTableEntry rightShooterForwardHardLimitFaultEntry;
  private final NetworkTableEntry rightShooterForwardSoftLimitFaultEntry;
  private final NetworkTableEntry rightShooterHardwareFaultEntry;
  private final NetworkTableEntry rightShooterMissingDifferentialFXFaultEntry;
  private final NetworkTableEntry rightShooterOverSupplyVFaultEntry;
  private final NetworkTableEntry rightShooterProcTempFaultEntry;
  private final NetworkTableEntry rightShooterReverseHardLimitFaultEntry;
  private final NetworkTableEntry rightShooterReverseSoftLimitFaultEntry;
  private final NetworkTableEntry rightShooterStatorCurrLimitFaultEntry;
  private final NetworkTableEntry rightShooterSupplyCurrLimitFaultEntry;
  private final NetworkTableEntry rightShooterUnlicensedFeatureInUseFaultEntry;
  private final NetworkTableEntry rightShooterUnstableSupplyVFaultEntry;

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

    /* Initialize NetworkTables entries */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");
    leftShooterFaultEntry = table.getEntry("LeftShooterFault");
    leftShooterUndervoltageFaultEntry = table.getEntry("LeftShooterUndervoltageFault");
    leftShooterBootDuringEnableFaultEntry = table.getEntry("LeftShooterBootDuringEnableFault");
    leftShooterBridgeBrownoutFaultEntry = table.getEntry("LeftShooterBridgeBrownoutFault");
    leftShooterDeviceTempFaultEntry = table.getEntry("LeftShooterDeviceTempFault");
    leftShooterForwardHardLimitFaultEntry = table.getEntry("LeftShooterForwardHardLimitFault");
    leftShooterForwardSoftLimitFaultEntry = table.getEntry("LeftShooterForwardSoftLimitFault");
    leftShooterHardwareFaultEntry = table.getEntry("LeftShooterHardwareFault");
    leftShooterMissingDifferentialFXFaultEntry =
        table.getEntry("LeftShooterMissingDifferentialFXFault");
    leftShooterOverSupplyVFaultEntry = table.getEntry("LeftShooterOverSupplyVFault");
    leftShooterProcTempFaultEntry = table.getEntry("LeftShooterProcTempFault");
    leftShooterReverseHardLimitFaultEntry = table.getEntry("LeftShooterReverseHardLimitFault");
    leftShooterReverseSoftLimitFaultEntry = table.getEntry("LeftShooterReverseSoftLimitFault");
    leftShooterStatorCurrLimitFaultEntry = table.getEntry("LeftShooterStatorCurrLimitFault");
    leftShooterSupplyCurrLimitFaultEntry = table.getEntry("LeftShooterSupplyCurrLimitFault");
    leftShooterUnlicensedFeatureInUseFaultEntry =
        table.getEntry("LeftShooterUnlicensedFeatureInUseFault");
    leftShooterUnstableSupplyVFaultEntry = table.getEntry("LeftShooterUnstableSupplyVFault");

    rightShooterFaultEntry = table.getEntry("RightShooterFault");
    rightShooterUndervoltageFaultEntry = table.getEntry("RightShooterUndervoltageFault");
    rightShooterBootDuringEnableFaultEntry = table.getEntry("RightShooterBootDuringEnableFault");
    rightShooterBridgeBrownoutFaultEntry = table.getEntry("RightShooterBridgeBrownoutFault");
    rightShooterDeviceTempFaultEntry = table.getEntry("RightShooterDeviceTempFault");
    rightShooterForwardHardLimitFaultEntry = table.getEntry("RightShooterForwardHardLimitFault");
    rightShooterForwardSoftLimitFaultEntry = table.getEntry("RightShooterForwardSoftLimitFault");
    rightShooterHardwareFaultEntry = table.getEntry("RightShooterHardwareFault");
    rightShooterMissingDifferentialFXFaultEntry =
        table.getEntry("RightShooterMissingDifferentialFXFault");
    rightShooterOverSupplyVFaultEntry = table.getEntry("RightShooterOverSupplyVFault");
    rightShooterProcTempFaultEntry = table.getEntry("RightShooterProcTempFault");
    rightShooterReverseHardLimitFaultEntry = table.getEntry("RightShooterReverseHardLimitFault");
    rightShooterReverseSoftLimitFaultEntry = table.getEntry("RightShooterReverseSoftLimitFault");
    rightShooterStatorCurrLimitFaultEntry = table.getEntry("RightShooterStatorCurrLimitFault");
    rightShooterSupplyCurrLimitFaultEntry = table.getEntry("RightShooterSupplyCurrLimitFault");
    rightShooterUnlicensedFeatureInUseFaultEntry =
        table.getEntry("RightShooterUnlicensedFeatureInUseFault");
    rightShooterUnstableSupplyVFaultEntry = table.getEntry("RightShooterUnstableSupplyVFault");
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

    // Push left shooter faults to NetworkTables
    leftShooterFaultEntry.setBoolean(left.getFault_DeviceTemp().getValue());
    leftShooterUndervoltageFaultEntry.setBoolean(left.getFault_Undervoltage().getValue());
    leftShooterBootDuringEnableFaultEntry.setBoolean(left.getFault_BootDuringEnable().getValue());
    leftShooterBridgeBrownoutFaultEntry.setBoolean(left.getFault_BridgeBrownout().getValue());
    leftShooterDeviceTempFaultEntry.setBoolean(left.getFault_DeviceTemp().getValue());
    leftShooterForwardHardLimitFaultEntry.setBoolean(left.getFault_ForwardHardLimit().getValue());
    leftShooterForwardSoftLimitFaultEntry.setBoolean(left.getFault_ForwardSoftLimit().getValue());
    leftShooterHardwareFaultEntry.setBoolean(left.getFault_Hardware().getValue());
    leftShooterMissingDifferentialFXFaultEntry.setBoolean(
        left.getFault_MissingDifferentialFX().getValue());
    leftShooterOverSupplyVFaultEntry.setBoolean(left.getFault_OverSupplyV().getValue());
    leftShooterProcTempFaultEntry.setBoolean(left.getFault_ProcTemp().getValue());
    leftShooterReverseHardLimitFaultEntry.setBoolean(left.getFault_ReverseHardLimit().getValue());
    leftShooterReverseSoftLimitFaultEntry.setBoolean(left.getFault_ReverseSoftLimit().getValue());
    leftShooterStatorCurrLimitFaultEntry.setBoolean(left.getFault_StatorCurrLimit().getValue());
    leftShooterSupplyCurrLimitFaultEntry.setBoolean(left.getFault_SupplyCurrLimit().getValue());
    leftShooterUnlicensedFeatureInUseFaultEntry.setBoolean(
        left.getFault_UnlicensedFeatureInUse().getValue());
    leftShooterUnstableSupplyVFaultEntry.setBoolean(left.getFault_UnstableSupplyV().getValue());

    // Push right shooter faults to NetworkTables
    rightShooterFaultEntry.setBoolean(right.getFault_DeviceTemp().getValue());
    rightShooterUndervoltageFaultEntry.setBoolean(right.getFault_Undervoltage().getValue());
    rightShooterBootDuringEnableFaultEntry.setBoolean(right.getFault_BootDuringEnable().getValue());
    rightShooterBridgeBrownoutFaultEntry.setBoolean(right.getFault_BridgeBrownout().getValue());
    rightShooterDeviceTempFaultEntry.setBoolean(right.getFault_DeviceTemp().getValue());
    rightShooterForwardHardLimitFaultEntry.setBoolean(right.getFault_ForwardHardLimit().getValue());
    rightShooterForwardSoftLimitFaultEntry.setBoolean(right.getFault_ForwardSoftLimit().getValue());
    rightShooterHardwareFaultEntry.setBoolean(right.getFault_Hardware().getValue());
    rightShooterMissingDifferentialFXFaultEntry.setBoolean(
        right.getFault_MissingDifferentialFX().getValue());
    rightShooterOverSupplyVFaultEntry.setBoolean(right.getFault_OverSupplyV().getValue());
    rightShooterProcTempFaultEntry.setBoolean(right.getFault_ProcTemp().getValue());
    rightShooterReverseHardLimitFaultEntry.setBoolean(right.getFault_ReverseHardLimit().getValue());
    rightShooterReverseSoftLimitFaultEntry.setBoolean(right.getFault_ReverseSoftLimit().getValue());
    rightShooterStatorCurrLimitFaultEntry.setBoolean(right.getFault_StatorCurrLimit().getValue());
    rightShooterSupplyCurrLimitFaultEntry.setBoolean(right.getFault_SupplyCurrLimit().getValue());
    rightShooterUnlicensedFeatureInUseFaultEntry.setBoolean(
        right.getFault_UnlicensedFeatureInUse().getValue());
    rightShooterUnstableSupplyVFaultEntry.setBoolean(right.getFault_UnstableSupplyV().getValue());
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

    if (RobotContainer.superstructure.getSystemState() == SuperstructureState.PRE_CLIMB) {
      if (velocityLeft > 0.0) {
        left.setControl(
            new VelocityVoltage(velocityLeft / 60.0)
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));
      } else {
        left.setControl(new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true));
      }

      if (velocityRight > 0.0) {
        right.setControl(
            new VelocityVoltage(velocityRight / 60.0)
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));
      } else {
        right.setControl(new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true));
      }
    } else {
      if (velocityLeft > 0.0) {
        left.setControl(
            new VelocityVoltage(velocityLeft / 60.0)
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false));
      } else {
        left.setControl(new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(false));
      }

      if (velocityRight > 0.0) {
        right.setControl(
            new VelocityVoltage(velocityRight / 60.0)
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false));
      } else {
        right.setControl(new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(false));
      }
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
