package frc.robot.commons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;

public class BreadUtil {

  // Private constructor so that the class cannot be instantiated
  private BreadUtil() {}

  // Returns the angle to the target
  public static Rotation2d getAngleToTarget(
      Translation2d fieldToRobot, Translation2d fieldToTarget) {
    Translation2d targetToRobot = fieldToTarget.minus(fieldToRobot);
    return new Rotation2d(targetToRobot.getX(), targetToRobot.getY());
  }

  // Converts a Rotation2d object to a double within the range of [0, 2pi]
  public static double getRadians0To2PI(Rotation2d angle) {
    return angle.getRadians() < 0.0 ? 2 * Math.PI + angle.getRadians() : angle.getRadians();
  }

  // Converts an angle in radians to a double within the range of [0, 2pi]
  public static double getRadians0to2PI(double angle) {
    angle %= (2 * Math.PI);
    if (angle < 0.0) {
      angle += (2 * Math.PI);
    }
    return angle;
  }

  // Returns the vector projection of one translation2d onto the other
  public static Translation2d vectorProjection(Translation2d u, Translation2d v) {
    double scalar = dot(u, v) / Math.pow(v.getNorm(), 2);
    return v.times(scalar);
  }

  // Returns the dot product of two translation2ds
  public static double dot(Translation2d u, Translation2d v) {
    return u.getX() * v.getX() + u.getY() * v.getY();
  }

  // Returns the FPGA timestamp in seconds
  public static double getFPGATimeSeconds() {
    return RobotController.getFPGATime() / 1.0E6;
  }

  // At reference method
  public static boolean atReference(
      double val, double reference, double tolerance, boolean inclusive) {
    return inclusive
        ? (Math.abs(reference - val) <= tolerance)
        : (Math.abs(reference - val) < tolerance);
  }

  // In bound method
  public static boolean inBound(
      double val, double lowerBound, double higherBound, boolean inclusive) {
    return inclusive
        ? (lowerBound <= val && higherBound >= val)
        : (lowerBound < val && higherBound > val);
  }

  // Deadband method
  public static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance) return 0.0;

    return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }
}
