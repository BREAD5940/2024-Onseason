package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTableRed {

  private InterpolatingTableRed() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(
                  Units.inchesToMeters(51.4), new ShotParameter(-50, 2800, 2000, 0.175)), // 3 feet
              entry(
                  Units.inchesToMeters(72.4), new ShotParameter(-42, 2800, 2000, 0.125)), // 5 feet
              entry(
                  Units.inchesToMeters(97.0),
                  new ShotParameter(-34.75, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(121.6), new ShotParameter(-30, 2800, 2000, 0.05)), // 9 feet
              entry(
                  Units.inchesToMeters(144.5), new ShotParameter(-28, 2800, 2000, 0.0)), // 11 feet
              entry(
                  Units.inchesToMeters(168.0),
                  new ShotParameter(-24.75, 2800, 2000, 0.0)), // 13 feet
              entry(
                  Units.inchesToMeters(192.0),
                  new ShotParameter(-23.75, 2800, 2000, 0.0)), // 15 feet
              entry(
                  Units.inchesToMeters(216.5),
                  new ShotParameter(-21.9, 2800, 2000, 0.0)), // 17 feet
              entry(
                  Units.inchesToMeters(242.0), new ShotParameter(-21, 3200, 2100, 0.0)), // 19 feet
              entry(
                  Units.inchesToMeters(264.0), new ShotParameter(-20, 3200, 2100, 0.0) // 21 feet
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
