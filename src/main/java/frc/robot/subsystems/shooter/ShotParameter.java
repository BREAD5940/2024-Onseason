package frc.robot.subsystems.shooter;

public class ShotParameter {

  public final double pivotAngleDeg;
  public final double leftRPM;
  public final double rightRPM;

  public ShotParameter(double pivotAngleDeg, double leftRPM, double rightRPM) {
    this.pivotAngleDeg = pivotAngleDeg;
    this.leftRPM = leftRPM;
    this.rightRPM = rightRPM;
  }

  public boolean equals(ShotParameter other) {
    return Math.abs(other.pivotAngleDeg - pivotAngleDeg) < 0.1
        && Math.abs(other.leftRPM - leftRPM) < 0.1
        && Math.abs(other.rightRPM - rightRPM) < 0.1;
  }

  public ShotParameter interpolate(ShotParameter end, double t) {
    return new ShotParameter(
        lerp(pivotAngleDeg, end.pivotAngleDeg, t),
        lerp(leftRPM, end.leftRPM, t),
        lerp(rightRPM, end.rightRPM, t));
  }

  private double lerp(double y1, double y2, double t) {
    return y1 + (t * (y2 - y1));
  }
}