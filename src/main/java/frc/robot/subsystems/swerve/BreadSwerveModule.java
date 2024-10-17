package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.Alert;
import frc.robot.commons.Alert.AlertType;

/**
 * Swerve Module class that encapsulates a swerve module powered by CTR Electronics devices.
 *
 * <p>This class handles the hardware devices and configures them for swerve module operation using
 * the Phoenix 6 API.
 *
 * <p>This class constructs hardware devices internally, so the user only specifies the constants
 * (IDs, PID gains, gear ratios, etc). Getters for these hardware devices are available.
 */
public class BreadSwerveModule {

  /** All possible control requests for the module steer motor. */
  public enum SteerRequestType {
    /**
     * Control the drive motor using a Motion Magic® request. The control output type is determined
     * by {@link SwerveModuleConstants#SteerMotorClosedLoopOutput}
     */
    MotionMagic,
    /**
     * Control the drive motor using a Motion Magic® Expo request. The control output type is
     * determined by {@link SwerveModuleConstants#SteerMotorClosedLoopOutput}
     */
    MotionMagicExpo,
  }

  /** All possible control requests for the module drive motor. */
  public enum DriveRequestType {
    /** Control the drive motor using an open-loop voltage request. */
    OpenLoopVoltage,
    /**
     * Control the drive motor using a velocity closed-loop request. The control output type is
     * determined by {@link SwerveModuleConstants#DriveMotorClosedLoopOutput}
     */
    Velocity,
  }

  /* Alerts */
  Alert isAliveAlertDrive = new Alert("Swerve Drive Motor is not Alive", AlertType.ERROR);
  Alert isOverheatingAlertDrive = new Alert("Swerve Drive Motor Motor Overheat", AlertType.WARNING);

  Alert isAliveAlertSteer = new Alert("Swerve Steer Motor is not Alive", AlertType.ERROR);
  Alert isOverheatingAlertSteer = new Alert("Swerve Steer Motor Motor Overheat", AlertType.WARNING);

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_cancoder;

  private final StatusSignal<Double> m_drivePosition;
  private final StatusSignal<Double> m_driveVelocity;
  private final StatusSignal<Double> m_steerPosition;
  private final StatusSignal<Double> m_steerVelocity;
  private final StatusSignal<Double> m_statorCurrent;
  private final StatusSignal<Double> m_supplyCurrent;
  private final BaseStatusSignal[] m_signals;
  private final double m_driveRotationsPerMeter;
  private final double m_couplingRatioDriveRotorToCANcoder;

  private final double m_speedAt12VoltsMps;

  /* steer motor controls */
  private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0);
  private final MotionMagicTorqueCurrentFOC m_angleTorqueSetter =
      new MotionMagicTorqueCurrentFOC(0);
  private final MotionMagicExpoVoltage m_angleVoltageExpoSetter = new MotionMagicExpoVoltage(0);
  private final MotionMagicExpoTorqueCurrentFOC m_angleTorqueExpoSetter =
      new MotionMagicExpoTorqueCurrentFOC(0);
  /* drive motor controls */
  private final VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0);
  private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);
  /* Velocity Torque current neutral should always be coast, as neutral corresponds to 0-current or maintain velocity, not 0-velocity */
  private final VelocityTorqueCurrentFOC m_velocityTorqueSetter =
      new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);

  private final ClosedLoopOutputType m_steerClosedLoopOutput;
  private final ClosedLoopOutputType m_driveClosedLoopOutput;

  private final SwerveModulePosition m_internalState = new SwerveModulePosition();
  private SwerveModuleState m_targetState = new SwerveModuleState();

  /**
   * Construct a SwerveModule with the specified constants.
   *
   * @param constants Constants used to construct the module
   * @param canbusName The name of the CAN bus this module is on
   */
  public BreadSwerveModule(SwerveModuleConstants constants, String canbusName) {
    m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
    m_steerMotor = new TalonFX(constants.SteerMotorId, canbusName);
    m_cancoder = new CANcoder(constants.CANcoderId, canbusName);

    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

    talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonConfigs.Slot0 = constants.DriveMotorGains;
    talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 80.0;
    talonConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    talonConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.02;
    talonConfigs.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.02;
    talonConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    talonConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
    talonConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    talonConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    talonConfigs.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    StatusCode response = m_driveMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID "
              + m_driveMotor.getDeviceID()
              + " failed config with error "
              + response.toString());
    }

    /* Undo changes for torqueCurrent */
    talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
    /* And to current limits */
    talonConfigs.CurrentLimits = new CurrentLimitsConfigs();

    talonConfigs.Slot0 = constants.SteerMotorGains;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    // Modify configuration to use remote CANcoder fused
    talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
    switch (constants.FeedbackSource) {
      case RemoteCANcoder:
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        break;
      case FusedCANcoder:
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        break;
      case SyncCANcoder:
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        break;
    }
    talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    talonConfigs.MotionMagic.MotionMagicAcceleration =
        talonConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    talonConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    talonConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;

    talonConfigs.ClosedLoopGeneral.ContinuousWrap =
        true; // Enable continuous wrap for swerve modules

    talonConfigs.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    response = m_steerMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID "
              + m_steerMotor.getDeviceID()
              + " failed config with error "
              + response.toString());
    }

    CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
    cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
    response = m_cancoder.getConfigurator().apply(cancoderConfigs);
    if (!response.isOK()) {
      System.out.println(
          "CANcoder ID "
              + m_cancoder.getDeviceID()
              + " failed config with error "
              + response.toString());
    }

    m_drivePosition = m_driveMotor.getPosition().clone();
    m_driveVelocity = m_driveMotor.getVelocity().clone();
    m_steerPosition = m_steerMotor.getPosition().clone();
    m_steerVelocity = m_steerMotor.getVelocity().clone();
    m_statorCurrent = m_driveMotor.getStatorCurrent().clone();
    m_supplyCurrent = m_driveMotor.getSupplyCurrent().clone();

    m_signals = new BaseStatusSignal[4];
    m_signals[0] = m_drivePosition;
    m_signals[1] = m_driveVelocity;
    m_signals[2] = m_steerPosition;
    m_signals[3] = m_steerVelocity;

    /* Calculate the ratio of drive motor rotation to meter on ground */
    double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
    double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
    m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    m_couplingRatioDriveRotorToCANcoder = constants.CouplingGearRatio;

    /* Make control requests synchronous */
    m_angleVoltageSetter.UpdateFreqHz = 0;
    m_angleTorqueSetter.UpdateFreqHz = 0;
    m_angleVoltageExpoSetter.UpdateFreqHz = 0;
    m_angleTorqueExpoSetter.UpdateFreqHz = 0;

    m_velocityTorqueSetter.UpdateFreqHz = 0;
    m_velocityVoltageSetter.UpdateFreqHz = 0;
    m_voltageOpenLoopSetter.UpdateFreqHz = 0;

    /* Set the drive motor closed-loop output type */
    m_steerClosedLoopOutput = constants.SteerMotorClosedLoopOutput;
    m_driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput;

    /* Get the expected speed when applying 12 volts */
    m_speedAt12VoltsMps = constants.SpeedAt12VoltsMps;

    /* Set status frame periods on the current signals */
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_statorCurrent, m_supplyCurrent);
  }

  /**
   * Gets the state of this module and passes it back as a SwerveModulePosition object with latency
   * compensated values.
   *
   * @param refresh True if the signals should be refreshed
   * @return SwerveModulePosition containing this module's state.
   */
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      m_drivePosition.refresh();
      m_driveVelocity.refresh();
      m_steerPosition.refresh();
      m_steerVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot =
        BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
    double angle_rot =
        BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

    /*
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    drive_rot -= angle_rot * m_couplingRatioDriveRotorToCANcoder;

    /* And push them into a SwerveModulePosition object to return */
    m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
    /* Angle is already in terms of steer rotations */
    m_internalState.angle = Rotation2d.fromRotations(angle_rot);

    return m_internalState;
  }

  /**
   * Applies the desired SwerveModuleState to this module.
   *
   * @param state Speed and direction the module should target
   * @param driveRequestType The {@link DriveRequestType} to apply
   */
  public void apply(SwerveModuleState state, DriveRequestType driveRequestType) {
    apply(state, driveRequestType, SteerRequestType.MotionMagic);
  }

  /**
   * Applies the desired SwerveModuleState to this module.
   *
   * @param state Speed and direction the module should target
   * @param driveRequestType The {@link DriveRequestType} to apply
   * @param steerRequestType The {@link SteerRequestType} to apply; defaults to {@link
   *     SteerRequestType#MotionMagic}
   */
  public void apply(
      SwerveModuleState state,
      DriveRequestType driveRequestType,
      SteerRequestType steerRequestType) {
    var optimized = SwerveModuleState.optimize(state, m_internalState.angle);
    m_targetState = optimized;

    double angleToSetDeg = optimized.angle.getRotations();
    switch (steerRequestType) {
      case MotionMagic:
        switch (m_steerClosedLoopOutput) {
          case Voltage:
            m_steerMotor.setControl(
                m_angleVoltageSetter.withPosition(angleToSetDeg).withEnableFOC(true));
            break;

          case TorqueCurrentFOC:
            m_steerMotor.setControl(m_angleTorqueSetter.withPosition(angleToSetDeg));
            break;
        }
        break;

      case MotionMagicExpo:
        switch (m_steerClosedLoopOutput) {
          case Voltage:
            m_steerMotor.setControl(
                m_angleVoltageExpoSetter.withPosition(angleToSetDeg).withEnableFOC(true));
            break;

          case TorqueCurrentFOC:
            m_steerMotor.setControl(m_angleTorqueExpoSetter.withPosition(angleToSetDeg));
            break;
        }
        break;
    }

    double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;

    /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
    /* To reduce the "skew" that occurs when changing direction */
    double steerMotorError = angleToSetDeg - m_steerPosition.getValue();
    /* If error is close to 0 rotations, we're already there, so apply full power */
    /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
    double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
    /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
    if (cosineScalar < 0.0) {
      cosineScalar = 0.0;
    }
    velocityToSet *= cosineScalar;

    /* Back out the expected shimmy the drive motor will see */
    /* Find the angular rate to determine what to back out */
    double azimuthTurnRps = m_steerVelocity.getValue();
    /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
    double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
    velocityToSet += driveRateBackOut;

    switch (driveRequestType) {
      case OpenLoopVoltage:
        /* Open loop ignores the driveRotationsPerMeter since it only cares about the open loop at the mechanism */
        /* But we do care about the backout due to coupling, so we keep it in */
        velocityToSet /= m_driveRotationsPerMeter;
        m_driveMotor.setControl(
            m_voltageOpenLoopSetter
                .withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0)
                .withEnableFOC(true));
        break;

      case Velocity:
        switch (m_driveClosedLoopOutput) {
          case Voltage:
            m_driveMotor.setControl(
                m_velocityVoltageSetter.withVelocity(velocityToSet).withEnableFOC(true));
            break;

          case TorqueCurrentFOC:
            m_driveMotor.setControl(m_velocityTorqueSetter.withVelocity(velocityToSet));
            break;
        }
        break;
    }
  }

  /**
   * Controls this module to the specified steer target, and applies the specific drive request.
   *
   * <p>This is intended only to be used for characterization of the robot, do not use this for
   * normal use.
   *
   * @param steerTarget The angle the wheels should face for characterization
   * @param driveRequest The direct voltage to apply to the motor for use during characterization
   */
  public void applyCharacterization(Rotation2d steerTarget, VoltageOut driveRequest) {
    double angleToSetDeg = steerTarget.getRotations();
    /* Use the configured closed loop output mode */
    switch (m_steerClosedLoopOutput) {
      case Voltage:
        m_steerMotor.setControl(m_angleVoltageSetter.withPosition(angleToSetDeg));
        break;

      case TorqueCurrentFOC:
        m_steerMotor.setControl(m_angleTorqueSetter.withPosition(angleToSetDeg));
        break;
    }

    /* And apply the high-level drive request */
    m_driveMotor.setControl(driveRequest);
  }

  /**
   * Controls this module to the specified steer target, and applies the specific drive request.
   *
   * <p>This is intended only to be used for characterization of the robot, do not use this for
   * normal use.
   *
   * @param steerTarget The angle the wheels should face for characterization
   * @param driveRequest The direct Torque Current to apply to the motor for use during
   *     characterization
   */
  public void applyCharacterization(Rotation2d steerTarget, TorqueCurrentFOC driveRequest) {
    double angleToSetDeg = steerTarget.getRotations();
    /* Use the configured closed loop output mode */
    switch (m_steerClosedLoopOutput) {
      case Voltage:
        m_steerMotor.setControl(m_angleVoltageSetter.withPosition(angleToSetDeg));
        break;

      case TorqueCurrentFOC:
        m_steerMotor.setControl(m_angleTorqueSetter.withPosition(angleToSetDeg));
        break;
    }

    /* And apply the high-level drive request */
    m_driveMotor.setControl(driveRequest);
  }

  /**
   * Configures the neutral mode to use for the module's drive motor.
   *
   * @param neutralMode The drive motor neutral mode
   * @return Status code response of the request
   */
  public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
    var configs = new MotorOutputConfigs();

    /* First read the configs so they're up-to-date */
    StatusCode status = m_driveMotor.getConfigurator().refresh(configs);
    if (status.isOK()) {
      /* Then set the neutral mode config to the appropriate value */
      configs.NeutralMode = neutralMode;
      status = m_driveMotor.getConfigurator().apply(configs);
    }
    if (!status.isOK()) {
      System.out.println(
          "TalonFX ID "
              + m_driveMotor.getDeviceID()
              + " failed config neutral mode with error "
              + status.toString());
    }
    return status;
  }

  /**
   * Gets the last cached swerve module position. This differs from {@link getPosition} in that it
   * will not perform any latency compensation or refresh the signals.
   *
   * @return Last cached SwerveModulePosition
   */
  public SwerveModulePosition getCachedPosition() {
    return m_internalState;
  }

  /**
   * Get the current state of the module.
   *
   * <p>This is typically used for telemetry, as the SwerveModulePosition is used for odometry.
   *
   * @return Current state of the module
   */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        m_driveVelocity.getValue() / m_driveRotationsPerMeter,
        Rotation2d.fromRotations(m_steerPosition.getValue()));
  }

  /**
   * Get the target state of the module.
   *
   * <p>This is typically used for telemetry.
   *
   * @return Target state of the module
   */
  public SwerveModuleState getTargetState() {
    return m_targetState;
  }

  /**
   * Gets the position/velocity signals of the drive and steer
   *
   * @return Array of BaseStatusSignals for this module in the following order: 0 - Drive Position 1
   *     - Drive Velocity 2 - Steer Position 3 - Steer Velocity
   */
  BaseStatusSignal[] getSignals() {
    return m_signals;
  }

  /** Resets this module's drive motor position to 0 rotations. */
  public void resetPosition() {
    /* Only touch drive pos, not steer */
    m_driveMotor.setPosition(0);
  }

  /**
   * Gets this module's Drive Motor TalonFX reference.
   *
   * <p>This should be used only to access signals and change configurations that the swerve
   * drivetrain does not configure itself.
   *
   * @return This module's Drive Motor reference
   */
  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  /**
   * Gets this module's Steer Motor TalonFX reference.
   *
   * <p>This should be used only to access signals and change configurations that the swerve
   * drivetrain does not configure itself.
   *
   * @return This module's Steer Motor reference
   */
  public TalonFX getSteerMotor() {
    return m_steerMotor;
  }

  /**
   * Gets this module's CANcoder reference.
   *
   * <p>This should be used only to access signals and change configurations that the swerve
   * drivetrain does not configure itself.
   *
   * @return This module's CANcoder reference
   */
  public CANcoder getCANcoder() {
    return m_cancoder;
  }

  /* Returns stator current */
  public double getStatorCurrent() {
    BaseStatusSignal.refreshAll(m_statorCurrent);
    return m_statorCurrent.getValue();
  }

  /* Returns supply current */
  public double getSupplyCurrent() {
    BaseStatusSignal.refreshAll(m_supplyCurrent);
    return m_supplyCurrent.getValue();
  }

  /* Check Alerts */
  public void checkFaultStatus() {
    isAliveAlertDrive.set(m_driveMotor.isAlive());
    isAliveAlertSteer.set(m_steerMotor.isAlive());
    isOverheatingAlertDrive.set(m_driveMotor.getFault_DeviceTemp().getValue());
    isOverheatingAlertSteer.set(m_steerMotor.getFault_DeviceTemp().getValue());
  }
}
