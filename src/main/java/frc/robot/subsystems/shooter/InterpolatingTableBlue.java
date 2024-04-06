package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTableBlue {

  private InterpolatingTableBlue() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(
                  Units.inchesToMeters(54.0), new ShotParameter(-50, 2800, 2000, 0.175)), // 3 feet
              entry(
                  Units.inchesToMeters(76.75), new ShotParameter(-42, 2800, 2000, 0.125)), // 5 feet
              entry(
                  Units.inchesToMeters(101.0),
                  new ShotParameter(-34.75, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(123.25), new ShotParameter(-30, 2800, 2000, 0.05)), // 9 feet
              entry(
                  Units.inchesToMeters(149.2), new ShotParameter(-28, 2800, 2000, 0.0)), // 11 feet
              entry(
                  Units.inchesToMeters(174.2),
                  new ShotParameter(-24.75, 2800, 2000, 0.0)), // 13 feet
              entry(
                  Units.inchesToMeters(198.75),
                  new ShotParameter(-23.75, 2800, 2000, 0.0)), // 15 feet
              entry(
                  Units.inchesToMeters(220.0),
                  new ShotParameter(-21.9, 2800, 2000, 0.0)), // 17 feet
              entry(
                  Units.inchesToMeters(242.0), new ShotParameter(-21, 3200, 2100, 0.0)), // 19 feet
              entry(
                  Units.inchesToMeters(265.0), new ShotParameter(-20, 3200, 2100, 0.0) // 21 feet
                  )));

  public static ShotParameter get(double distanceToTarget) {
    Entry<Double, ShotParameter> ceil = table.ceilingEntry(distanceToTarget);
    Entry<Double, ShotParameter> floor = table.floorEntry(distanceToTarget);
    if (ceil == null) return floor.getValue();
    if (floor == null) return ceil.getValue();
    if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
    return floor
        .getValue()
        .interpolate(
            ceil.getValue(),
            (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey()));
  }
}
