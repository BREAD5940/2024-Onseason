package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class Shooter {

  /* IO and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

  /* Stops the shooter without going backwards */
  public void stop() {
    io.setFlywheelCurrentLimit(0, 0, 0);
    io.setFlywheelVelocity(0, 0);
    io.enableFlywheelBrakeMode(false);
  }

}
