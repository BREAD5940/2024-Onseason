package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class Intake {

  /* IO and input classes */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
    io.enableBrakeMode(false);
  }

  /* Method to be called periodically by Superstructure.java */
  public void onLoop() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    io.updateTunableNumbers();
  }

  /* Stop spinning the intake */
  public void stop() {
    io.setCurrentLimit(0, 0, 0);
    io.setPercent(0.0);
    io.enableBrakeMode(false);
  }
  
  /* Spin the intake at intaking speed */
  public void intake() {
    io.setCurrentLimit(0, 0, 0);
    io.setPercent(0);
    io.enableBrakeMode(false);
  }

  /* Spin the intake at spit speed */
  public void spit() {
    io.setCurrentLimit(0, 0, 0);
    io.setPercent(0);
    io.enableBrakeMode(false);
  }

  /* Hold a game piece */
  public void hold() {
    io.setCurrentLimit(0, 0, 0);
    io.setPercent(0);
    io.enableBrakeMode(false);
  }

  /* Returns the speed of the intake motor in rpm */
  public double getMotorSpeedRPM() {
    return inputs.velocityRpm;
  }

  /* Returns whether the top beam break on the intake is triggered */
  public boolean getTopBeamBreakTriggered() {
    return inputs.topBeamBreakTriggered;
  }

  /* Returns whether the bottom beam break on the intake is triggered */
  public boolean getBottomBeamBreakTriggered() {
    return inputs.bottomBeamBreakTriggered;
  }

}
