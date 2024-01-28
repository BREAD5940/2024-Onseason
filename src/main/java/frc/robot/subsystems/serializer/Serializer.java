package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.Logger;

public class Serializer {

  /* IO and input classes */
  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();

  public Serializer(SerializerIO io) {
    this.io = io;
    io.enableBrakeMode(false);
  }

  /* Method to be called periodically by Superstructure.java */
  public void onLoop() {
    io.updateInputs(inputs);
    Logger.processInputs("Serializer", inputs);
    io.updateTunableNumbers();
  }

  /* Request the serializer to stop */
  public void stop() {
    io.setCurrentLimit(0, 0, 0);
    io.setPercent(0);
    io.enableBrakeMode(false);
  }

  /* Request the serializer to hold a game piece */
  public void hold() {
    io.setCurrentLimit(0, 0, 0);
    io.setVelocity(0.0);
    io.enableBrakeMode(true);
  }

  /* Request the serializer to index a game piece */
  public void index() {
    io.setCurrentLimit(0, 0, 0);
    io.setVelocity(0);
    io.enableBrakeMode(true);
  }

  /* Requests the serializer to run in reverse */
  public void spit() {
    io.setCurrentLimit(0, 0, 0);
    io.setVelocity(0);
    io.enableBrakeMode(false);
  }

  /* Returns whether the beam break in the serializer has been triggered */
  public boolean getBeamBreakTriggered() {
    return inputs.beamBreakTriggered;
  }
}
