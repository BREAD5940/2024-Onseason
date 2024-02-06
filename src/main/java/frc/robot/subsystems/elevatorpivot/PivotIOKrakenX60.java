package frc.robot.subsystems.elevatorpivot;

import static frc.robot.constants.Constants.Pivot.*;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;

public class PivotIOKrakenX60 implements PivotIO {

  /* Hardware */
  private final TalonFX pivot = new TalonFX(PIVOT_ID, "dabus");
  private final CANcoder azimuth = new CANcoder(PIVOT_AZIMUTH_ID, "dabus");

  /* Configurators */
  private TalonFXConfigurator pivotConfigurator;
  private CANcoderConfigurator azimuthConfigurator;

  private final CurrentLimitsConfigs currentLimitConfigs;
  private final MotorOutputConfigs motorOutputConfigs;
  private final Slot0Configs slot0Configs;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", 0.0);
  LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/kG", 0.0);
  LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Pivot/kV", 12.0 / PIVOT_MAX_SPEED);
  LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", 10);
  LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", 0.0);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("ShooterPivot/MotionAcceleration", 2.5);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("ShooterPivot/MotionCruiseVelocity", 1.0);
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("ShooterPivot/MotionJerk", 0.0);

  public PivotIOKrakenX60() {
    /* Configurators */
    pivotConfigurator = pivot.getConfigurator();
    azimuthConfigurator = azimuth.getConfigurator();

    /* Configure pivot hardware */
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = 250.0;
    currentLimitConfigs.SupplyCurrentLimit = 250.0;

    motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = PIVOT_INVERSION;
    motorOutputConfigs.PeakForwardDutyCycle = 1.0;
    motorOutputConfigs.PeakReverseDutyCycle = -1.0;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS.get();
    slot0Configs.kG = kG.get();
    slot0Configs.kV = kV.get();
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    MagnetSensorConfigs pivotMagnetSensorConfigs = new MagnetSensorConfigs();
    pivotMagnetSensorConfigs.SensorDirection = PIVOT_ENCODER_INVERSION;
    pivotMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    pivotMagnetSensorConfigs.MagnetOffset = PIVOT_MAGNET_OFFSET;

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotFeedbackConfigs.FeedbackRemoteSensorID = azimuth.getDeviceID();
    pivotFeedbackConfigs.RotorToSensorRatio = PIVOT_GEAR_RATIO;
    pivotFeedbackConfigs.SensorToMechanismRatio = 1.0;

    MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs();
    pivotMotionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    pivotMotionMagicConfigs.MotionMagicJerk = motionJerk.get();

    /* Apply Configurations */
    azimuthConfigurator.apply(pivotMagnetSensorConfigs);
    pivotConfigurator.apply(currentLimitConfigs);
    pivotConfigurator.apply(motorOutputConfigs);
    pivotConfigurator.apply(slot0Configs);
    pivotConfigurator.apply(pivotFeedbackConfigs);
    pivotConfigurator.apply(pivotMotionMagicConfigs);

    pivot.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angleDegrees = Units.rotationsToDegrees(azimuth.getAbsolutePosition().getValue());
    inputs.angleRads = Units.rotationsToRadians(azimuth.getAbsolutePosition().getValue());
    inputs.velDegreesPerSecond = Units.rotationsToDegrees(pivot.getVelocity().getValue());
    inputs.currentAmps = pivot.getSupplyCurrent().getValue();
    inputs.appliedVoltage = pivot.getMotorVoltage().getValue();
    inputs.tempCelcius = pivot.getDeviceTemp().getValue();
    inputs.armTargetPosition = pivot.getClosedLoopReference().getValue();
    inputs.armTargetVelocity = pivot.getClosedLoopReferenceSlope().getValue();
  }

  @Override
  public void setPercent(double percent) {
    pivot.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setAngle(Rotation2d angle) {
    double setpoint =
        MathUtil.clamp(
            angle.getRotations(), PIVOT_MIN_ANGLE.getRotations(), PIVOT_MAX_ANGLE.getRotations());
    pivot.setControl(new MotionMagicVoltage(setpoint));
  }

  @Override
  public void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = currentLimit;
    currentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    currentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;

    pivotConfigurator.apply(currentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    pivotConfigurator.apply(motorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kG.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {
      slot0Configs.kS = kS.get();
      slot0Configs.kG = kG.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      pivotConfigurator.apply(slot0Configs);
    }
  }
}
