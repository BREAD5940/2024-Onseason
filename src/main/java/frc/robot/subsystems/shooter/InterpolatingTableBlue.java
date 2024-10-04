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
                  Units.inchesToMeters(55.52922002054039),
                  new ShotParameter(-50, 2800, 2000, 0.2)), // 3 feet
              entry(
                  Units.inchesToMeters(76.66599848060716),
                  new ShotParameter(-42, 2800, 2000, 0.15)), // 5 feet
              entry(
                  Units.inchesToMeters(99.67631278834472),
                  new ShotParameter(-34, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(122.4021877148048),
                  new ShotParameter(-28.5, 2800, 2000, 0.015)), // 9 feet
              entry(
                  Units.inchesToMeters(147.75119260435238),
                  new ShotParameter(-25.5, 2800, 2000, 0.015)), // 11 feet
              entry(
                  Units.inchesToMeters(172.26747221580885),
                  new ShotParameter(-23.5, 2800, 2000, 0.015)), // 13 feet
              entry(
                  Units.inchesToMeters(196.5281843936359), // 12
                  new ShotParameter(-21, 2800, 2000, 0.015)), // 15 feet
              entry(
                  Units.inchesToMeters(221.60080048571137), // 14
                  new ShotParameter(-19.5, 2800, 2000, 0.015)), // 17 feet
              entry(
                  Units.inchesToMeters(244.09283987001777), // 16
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
