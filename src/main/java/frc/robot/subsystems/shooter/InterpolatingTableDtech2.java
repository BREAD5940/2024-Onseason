package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTableDtech2 {

  private InterpolatingTableDtech2() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(Units.inchesToMeters(51.4), new ShotParameter(-50, 2800, 2000, 0.2)), // 3 feet
              entry(
                  Units.inchesToMeters(77.61319039029621),
                  new ShotParameter(-41.4, 2800, 2000, 0.15)), // 5 feet
              entry(
                  Units.inchesToMeters(100.69431214926642),
                  new ShotParameter(-35.5, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(123.89974010548269),
                  new ShotParameter(-31.5, 2800, 2000, 0.015)), // 9 feet
              entry(
                  Units.inchesToMeters(147.25942280925787),
                  new ShotParameter(-27.5, 2800, 2000, 0.015)), // 11 feet
              entry(
                  Units.inchesToMeters(170.90243984360367),
                  new ShotParameter(-26.2, 2800, 2000, 0.015)), // 13 feet
              entry(
                  Units.inchesToMeters(195.0),
                  new ShotParameter(-24.5, 2800, 2000, 0.015)), // 15 feet
              entry(
                  Units.inchesToMeters(215.0),
                  new ShotParameter(-22.7, 2800, 2000, 0.015)), // 17 feet
              entry(
                  Units.inchesToMeters(243.7),
                  new ShotParameter(-22, 3200, 2100, 0.015)) // 19 feet this one
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
