package frc.robot.subsystems.elevatorpivot;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class IntakeInterpolation {

  private IntakeInterpolation() {}

  public static TreeMap<Double, IntakeParameter> table =
      new TreeMap<>(Map.ofEntries(entry(0.1, new IntakeParameter(0))));

  public static IntakeParameter get(double elevatorHeight) {
    Entry<Double, IntakeParameter> ceil = table.ceilingEntry(elevatorHeight);
    Entry<Double, IntakeParameter> floor = table.floorEntry(elevatorHeight);
    if (ceil == null) return floor.getValue();
    if (floor == null) return ceil.getValue();
    if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
    return floor
        .getValue()
        .interpolate(
            ceil.getValue(), (elevatorHeight - floor.getKey()) / (ceil.getKey() - floor.getKey()));
  }

  static class IntakeParameter {

    public final double pivotAngleDeg;

    public IntakeParameter(double pivotAngleDeg) {
      this.pivotAngleDeg = pivotAngleDeg;
    }

    public boolean equals(IntakeParameter other) {
      return Math.abs(other.pivotAngleDeg - pivotAngleDeg) < 0.1;
    }

    public IntakeParameter interpolate(IntakeParameter end, double t) {
      return new IntakeParameter(lerp(pivotAngleDeg, end.pivotAngleDeg, t));
    }

    private double lerp(double y1, double y2, double t) {
      return y1 + (t * (y2 - y1));
    }
  }
}
