package frc.robot.subsystems.elevatorpivot;

import static frc.robot.constants.Constants.Elevator.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commons.LoggedTunableNumber;

public class ElevatorIOKrakenX60 implements ElevatorIO {

  /* Hardware */
  private final TalonFX leader;
  private final TalonFX follower;

  /* Configurators */
  private TalonFXConfigurator leaderConfigurator;
  private TalonFXConfigurator followerConfigurator;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final MotorOutputConfigs leaderMotorConfigs;
  private final MotorOutputConfigs followerMotorConfigs;
  private final Slot0Configs slot0Configs;
  private double posTarget;

  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.3);
  LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", 12.0 / metersToRotations(ELEVATOR_MAX_SPEED));
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 10);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Elevator/MotionAcceleration", metersToRotations(3.0));
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Elevator/MotionCruiseVelocity", metersToRotations(3.0));
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("Elevator/MotionJerk", 0.0);

  /* For tracking */

  public ElevatorIOKrakenX60() {
    /* Instantiate motors and configurators */
    this.leader = new TalonFX(ELEVATOR_LEFT_ID, "dabus");
    this.follower = new TalonFX(ELEVATOR_RIGHT_ID, "dabus");

    this.leaderConfigurator = leader.getConfigurator();
    this.followerConfigurator = follower.getConfigurator();

    /* Create configs */
    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 100.0;
    currentLimitsConfigs.SupplyCurrentLimit = 100.0;
    currentLimitsConfigs.SupplyTimeThreshold = 1.5;

    leaderMotorConfigs = new MotorOutputConfigs();
    leaderMotorConfigs.Inverted = ELEVATOR_LEFT_INVERSION;
    leaderMotorConfigs.PeakForwardDutyCycle = 1.0;
    leaderMotorConfigs.PeakReverseDutyCycle = -1.0;
    leaderMotorConfigs.NeutralMode = NeutralModeValue.Coast;

    followerMotorConfigs = new MotorOutputConfigs();
    followerMotorConfigs.PeakForwardDutyCycle = 1.0;
    followerMotorConfigs.PeakReverseDutyCycle = -1.0;
    followerMotorConfigs.NeutralMode = NeutralModeValue.Coast;

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    slot0Configs = new Slot0Configs();
    slot0Configs.kA = kA.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    /* Apply configs */
    leaderConfigurator.apply(currentLimitsConfigs);
    leaderConfigurator.apply(leaderMotorConfigs);
    leaderConfigurator.apply(slot0Configs);
    leaderConfigurator.apply(motionMagicConfigs);
    leaderConfigurator.apply(openLoopRampsConfigs);
    leaderConfigurator.apply(closedLoopRampsConfigs);

    followerConfigurator.apply(currentLimitsConfigs);
    followerConfigurator.apply(leaderMotorConfigs);
    followerConfigurator.apply(slot0Configs);
    followerConfigurator.apply(motionMagicConfigs);
    followerConfigurator.apply(openLoopRampsConfigs);
    followerConfigurator.apply(closedLoopRampsConfigs);

    follower.setControl(new Follower(ELEVATOR_LEFT_ID, false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.posMeters = getHeight();
    inputs.velMetersPerSecond = getVelocity();
    inputs.motionMagicVelocityTarget =
        rotationsToMeters(leader.getClosedLoopReferenceSlope().getValue());
    inputs.motionMagicPositionTarget =
        rotationsToMeters(leader.getClosedLoopReference().getValue());
    inputs.appliedVoltage = leader.getMotorVoltage().getValue();
    inputs.currentAmps =
        new double[] {leader.getStatorCurrent().getValue(), follower.getStatorCurrent().getValue()};
    inputs.tempCelcius =
        new double[] {leader.getStatorCurrent().getValue(), follower.getStatorCurrent().getValue()};
    inputs.setpointMeters = posTarget;
  }

  @Override
  public void setHeight(double heightMeters) {
    if (!DriverStation.isEnabled()) {
      leader.setControl(new VoltageOut(0.0));
      return;
    }
    posTarget = heightMeters;
    // double kG = getHeight() < ELEVATOR_S2_HEIGHT ? ELEVATOR_S1_KG : ELEVATOR_S2_KG;
    // double arbFF = MathUtil.clamp(kG, -0.999, 0.999);
    leader.setControl(new MotionMagicVoltage(metersToRotations(heightMeters)));
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void resetHeight(double newHeightMeters) {
    leader.setPosition(metersToRotations(newHeightMeters));
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    leaderMotorConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    followerMotorConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    leaderConfigurator.apply(leaderMotorConfigs);
    followerConfigurator.apply(followerMotorConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (kA.hasChanged(0)
        || kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      leaderConfigurator.apply(slot0Configs);
      followerConfigurator.apply(slot0Configs);
    }
  }

  private double getHeight() {
    return rotationsToMeters(leader.getPosition().getValueAsDouble());
  }

  private double getVelocity() {
    return rotationsToMeters(leader.getVelocity().getValueAsDouble());
  }

  private double metersToRotations(double heightMeters) {
    return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
  }

  private double rotationsToMeters(double rotations) {
    return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
  }
}
