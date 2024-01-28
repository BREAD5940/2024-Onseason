package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.Shooter.*;

import frc.robot.commons.BreadUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter {

  /* IO and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double desiredLeftRPM = 0.0;
  private double desiredRightRPM = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  /* Method to be called periodically by Superstructure.java */
  public void onLoop() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    io.enableFlywheelBrakeMode(false);
  }

  public void idleSpeed() {
    io.setFlywheelCurrentLimit(0, 0, 0);
    io.setFlywheelVelocity(0, 0);
    io.enableFlywheelBrakeMode(false);
  }

  /* Sets the shooter to the specified velocity on the left and right side */
  public void set(double left, double right) {
    io.setFlywheelCurrentLimit(0, 0, 0);
    io.setFlywheelVelocity(left, right);
    io.enableFlywheelBrakeMode(false);
  }

  /* Returns whether or not the shooter is at its setpoint */
  public boolean atSetpoint() {
    return BreadUtil.atReference(
            inputs.flywheelLeftVelocityRPM, desiredLeftRPM, SHOOTER_SETPOINT_TOLERANCE_RPM, false)
        && BreadUtil.atReference(
            inputs.flywheelRightVelocityRPM,
            desiredRightRPM,
            SHOOTER_SETPOINT_TOLERANCE_RPM,
            false);
  }

  /* Stops the shooter without going backwards */
  public void stop() {
    io.setFlywheelCurrentLimit(0, 0, 0);
    io.setFlywheelVelocity(0, 0);
    io.enableFlywheelBrakeMode(false);
  }
}
