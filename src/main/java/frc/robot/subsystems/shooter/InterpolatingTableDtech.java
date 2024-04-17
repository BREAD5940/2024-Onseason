package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTableDtech {

  private InterpolatingTableDtech() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(Units.inchesToMeters(51.4), new ShotParameter(-50, 2800, 2000, 0.2)), // 3 feet
              entry(Units.inchesToMeters(74.7), new ShotParameter(-42, 2800, 2000, 0.15)), // 5 feet
              entry(Units.inchesToMeters(97.0), new ShotParameter(-34, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(122.0),
                  new ShotParameter(-28.5, 2800, 2000, 0.015)), // 9 feet
              entry(
                  Units.inchesToMeters(146.0),
                  new ShotParameter(-25.5, 2800, 2000, 0.015)), // 11 feet
              entry(
                  Units.inchesToMeters(168.0),
                  new ShotParameter(-23.5, 2800, 2000, 0.015)), // 13 feet
              entry(
                  Units.inchesToMeters(190.0),
                  new ShotParameter(-21, 2800, 2000, 0.015)), // 15 feet
              entry(
                  Units.inchesToMeters(217.0),
                  new ShotParameter(-19.5, 2800, 2000, 0.015)), // 17 feet
              entry(
                  Units.inchesToMeters(243.5),
                  new ShotParameter(-18.25, 3200, 2100, 0.015)) // 19 feet
              ));

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
