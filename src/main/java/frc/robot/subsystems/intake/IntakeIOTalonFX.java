package frc.robot.subsystems.intake;

import static frc.robot.constants.Constants.Intake.*;

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
import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
  // create motor
   /* Hardware */
  private final TalonFX motor = new TalonFX(INTAKE_ID, "dabus");

  /* Status Signals */
  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;

  /* Configurator */
  private final TalonFXConfigurator configurator;

  /* Configs */
  private CurrentLimitsConfigs currentLimitConfigs;
  private Slot0Configs slot0Configs;
  private MotorOutputConfigs motorOutputConfigs;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.0);
  LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.0);
  LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);

  public IntakeIOTalonFX() {
    // create configuration
    configurator = motor.getConfigurator();
    // supply current limit
    currentLimitConfigs = new CurrentLimitsConfigs();

    currentLimitConfigs.SupplyCurrentLimit = 40.0;
    currentLimitConfigs.SupplyCurrentThreshold = 60.0;
    currentLimitConfigs.SupplyTimeThreshold = 1;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    // motor direction
  motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.PeakForwardDutyCycle = 1.0;
    motorOutputConfigs.PeakReverseDutyCycle = -1.0;


    // set motor to brake mode
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    // apply configuration to motor
    slot0Configs = new Slot0Configs();

    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();

    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    configurator.apply(currentLimitConfigs);
    configurator.apply(motorOutputConfigs);
    configurator.apply(slot0Configs);

    position = motor.getPosition();
    velocity = motor.getVelocity();

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // update positionRad, velocityRpm, appliedVolts, and currentAmps
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRPM = velocity.getValueAsDouble() * 60;
    inputs.appliedVolts = motor.getMotorVoltage().getValue();
    inputs.currentAmps = motor.getSupplyCurrent().getValue();
  } 
  
  @Override
  public void setPercent(double percent) {
  motor.setControl(new DutyCycleOut(percent));
  }
}