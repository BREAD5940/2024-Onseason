package frc.robot.subsystems.elevatorpivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;
import static frc.robot.constants.Constants.Pivot.*;

public class PivotIOTalonFX implements PivotIO {

  /* Hardware */
  private final TalonFX pivot = new TalonFX(PIVOT_ID, "dabus");
  private final CANcoder pivotAzimuth = new CANcoder(PIVOT_AZIMUTH_ID, "dabus");

  /* Status signals */
  private final StatusSignal<Double> pivotPosition = pivot.getPosition();
  private final StatusSignal<Double> pivotVelocity = pivot.getVelocity();

  /* Configurators */
  private TalonFXConfigurator pivotConfigurator;
  private CANcoderConfigurator pivotAzimuthConfigurator;

  private final CurrentLimitsConfigs pivotCurrentLimitConfigs;
  private final MotorOutputConfigs pivotMotorOutputConfigs;
  private final Slot0Configs pivotSlot0Configs;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("ShooterPivot/kS", 0.0);
  LoggedTunableNumber kG = new LoggedTunableNumber("ShooterPivot/kG", 0.0);
  LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "ShooterPivot/kV", 12.0 / ((6380.0 / 60.0) * (1.0 / PIVOT_GEAR_RATIO)));
  LoggedTunableNumber kP = new LoggedTunableNumber("ShooterPivot/kP", 10);
  LoggedTunableNumber kI = new LoggedTunableNumber("ShooterPivot/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("ShooterPivot/kD", 0.0);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("ShooterPivot/MotionAcceleration", 2.5);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("ShooterPivot/MotionCruiseVelocity", 1.0);
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("ShooterPivot/MotionJerk", 0.0);

  public PivotIOTalonFX() {
    /* Configurators */
    pivotConfigurator = pivot.getConfigurator();
    pivotAzimuthConfigurator = pivotAzimuth.getConfigurator();

    /* Configure pivot hardware */
    pivotCurrentLimitConfigs = new CurrentLimitsConfigs();
    pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    pivotCurrentLimitConfigs.StatorCurrentLimit = 250.0;
    pivotCurrentLimitConfigs.SupplyCurrentLimit = 250.0;

    pivotMotorOutputConfigs = new MotorOutputConfigs();
    pivotMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    pivotMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    pivotMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    pivotSlot0Configs = new Slot0Configs();
    pivotSlot0Configs.kS = kS.get();
    pivotSlot0Configs.kG = kG.get();
    pivotSlot0Configs.kV = kV.get();
    pivotSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    pivotSlot0Configs.kP = kP.get();
    pivotSlot0Configs.kI = kI.get();
    pivotSlot0Configs.kD = kD.get();

    MagnetSensorConfigs pivotMagnetSensorConfigs = new MagnetSensorConfigs();
    pivotMagnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    pivotMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    pivotMagnetSensorConfigs.MagnetOffset = PIVOT_MAGNET_OFFSET;

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotFeedbackConfigs.FeedbackRemoteSensorID = pivotAzimuth.getDeviceID();
    pivotFeedbackConfigs.RotorToSensorRatio = PIVOT_GEAR_RATIO;
    pivotFeedbackConfigs.SensorToMechanismRatio = 1.0;

    MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs();
    pivotMotionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    pivotMotionMagicConfigs.MotionMagicJerk = motionJerk.get();

    /* Apply Configurations */
    pivotAzimuthConfigurator.apply(pivotMagnetSensorConfigs);
    pivotConfigurator.apply(pivotCurrentLimitConfigs);
    pivotConfigurator.apply(pivotMotorOutputConfigs);
    pivotConfigurator.apply(pivotSlot0Configs);
    pivotConfigurator.apply(pivotFeedbackConfigs);
    pivotConfigurator.apply(pivotMotionMagicConfigs);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity);
    pivot.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angleDegrees =
        Units.rotationsToDegrees(pivotAzimuth.getAbsolutePosition().getValue());
    inputs.angleRads = Units.rotationsToRadians(pivotAzimuth.getAbsolutePosition().getValue());
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
    pivot.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setAngle(Rotation2d angle) {
    double setpoint = MathUtil.clamp(angle.getRotations(), PIVOT_MIN_ANGLE.getRotations(), PIVOT_MAX_ANGLE.getRotations());
    pivot.setControl(new MotionMagicVoltage(setpoint));
  }

  @Override
  public void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    pivotCurrentLimitConfigs.StatorCurrentLimit = currentLimit;
    pivotCurrentLimitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
    pivotCurrentLimitConfigs.SupplyTimeThreshold = supplyTimeThreshold;

    pivotConfigurator.apply(pivotCurrentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    pivotMotorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    pivotConfigurator.apply(pivotMotorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kG.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {
      pivotSlot0Configs.kS = kS.get();
      pivotSlot0Configs.kG = kG.get();
      pivotSlot0Configs.kV = kV.get();
      pivotSlot0Configs.kP = kP.get();
      pivotSlot0Configs.kI = kI.get();
      pivotSlot0Configs.kD = kD.get();

      pivotConfigurator.apply(pivotSlot0Configs);

      System.out.println("Configurations Applied!");
    }
  }
}