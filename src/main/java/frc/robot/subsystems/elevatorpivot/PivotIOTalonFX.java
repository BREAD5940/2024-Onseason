package frc.robot.subsystems.elevatorpivot;

import static frc.robot.constants.Constants.Shooter.*;

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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.wpilibj.CAN;
import frc.robot.commons.LoggedTunableNumber;
import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.Pivot.*;


/* Interface encapsulating pivot hardware */
public class PivotIOTalonFX implements PivotIO {

    private final TalonFX pivot = new TalonFX(PIVOT_ID, CANBUS_NAME);
  private final StatusSignal<Double> pivotPosition = pivot.getPosition();
  private final StatusSignal<Double> pivotVelocity = pivot.getVelocity();
  private TalonFXConfigurator pivotConfigurator;
  private CANcoderConfigurator pivotAzimuthConfigurator;
  private final CurrentLimitsConfigs pivotCurrentLimitConfigs;
  private final MotorOutputConfigs pivotMotorConfigs;
  private final Slot0Configs pivotSlot0Configs;
  private final CANcoder pivotAzimuth = new CANcoder(PIVOT_AZIMUTH_ID, CANBUS_NAME)

  LoggedTunableNumber pivotkS = new LoggedTunableNumber("ShooterPivot/kS", 0.0);
  LoggedTunableNumber pivotkG = new LoggedTunableNumber("ShooterPivot/kG", 0.0);
  LoggedTunableNumber pivotkV =
      new LoggedTunableNumber(
          "ShooterPivot/kV", 12.0 / ((6380.0 / 60.0) * (1.0 / PIVOT_GEAR_RATIO)));
  LoggedTunableNumber pivotkP = new LoggedTunableNumber("ShooterPivot/kP", 10);
  LoggedTunableNumber pivotkI = new LoggedTunableNumber("ShooterPivot/kI", 0.0);
  LoggedTunableNumber pivotKD = new LoggedTunableNumber("ShooterPivot/kD", 0.0);

  LoggedTunableNumber pivotMotionAcceleration =
      new LoggedTunableNumber("ShooterPivot/MotionAcceleration", 2.5);
  LoggedTunableNumber pivotMotionCruiseVelocity =
      new LoggedTunableNumber("ShooterPivot/MotionCruiseVelocity", 1.0);
  LoggedTunableNumber pivotMotionJerk = new LoggedTunableNumber("ShooterPivot/MotionJerk", 0.0);

  private double velocitySetpointRPM = 0.0;

  public PivotIOTalonFX() {
    //configurators
    pivotConfigurator = pivot.getConfigurator();
    pivotAzimuthConfigurator = pivotAzimuth.getConfigurator();
// current limits
    pivotCurrentLimitConfigs = new CurrentLimitsConfigs();
    pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    pivotCurrentLimitConfigs.StatorCurrentLimit = 250.0;
    pivotCurrentLimitConfigs.SupplyCurrentLimit = 250.0;
//motor output
    pivotMotorConfigs = new MotorOutputConfigs();
    pivotMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotorConfigs.PeakForwardDutyCycle = 1.0;
    pivitMotorConfigs.PeakReverseDutyCycle = -1.0;
    pivotMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    pivotSlot0Configs = new Slot0Configs();
    pivotSlot0Configs.kS = pivotkS.get();
    pivotSlot0Configs.kG = pivotkG.get();
    pivotSlot0Configs.kV = pivotkV.get();
    pivotSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    pivotSlot0Configs.kP = pivotkP.get();
    pivotSlot0Configs.kI = pivotkI.get();
    pivotSlot0Configs.kD = pivotKD.get();

    MagnetSensorConfigs pivotMagnetSensorConfigs = new MagnetSensorConfigs();
    pivotMagnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    pivotMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    pivotMagnetSensorConfigs.MagnetOffset = -0.139404;

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotFeedbackConfigs.FeedbackRemoteSensorID = pivotAzimuth.getDeviceID();
    pivotFeedbackConfigs.RotorToSensorRatio = PIVOT_GEAR_RATIO;
    pivotFeedbackConfigs.SensorToMechanismRatio = 1.0;

    MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs();
    pivotMotionMagicConfigs.MotionMagicAcceleration = pivotMotionAcceleration.get();
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = pivotMotionCruiseVelocity.get();
    pivotMotionMagicConfigs.MotionMagicJerk = pivotMotionJerk.get();



  }
  /** Updates the set of loggable inputs */
  public void updateInputs(PivotIOInputs inputs) {
  

  }

  /** Sets the desired angle of the pivot */
  public void setAngle(Rotation2d angle) {

  }

  /** Sets the speed of the pivot to the desired percent output */
  public void setPercent(double percent) {

  }

  /** Enables or disables the pivot in brake mode */
  public void enableBrakeMode(boolean enable) {

  }

  /** Updates tunable numbers */
  public void updateTunableNumbers() {

  }
}
