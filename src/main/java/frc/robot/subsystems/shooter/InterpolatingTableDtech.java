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
              entry(
                  Units.inchesToMeters(47.6),
                  new ShotParameter(-54.0, 1700, 1200, 0.33)), // (first) 3 feet
              entry(
                  Units.inchesToMeters(69.4), new ShotParameter(-44.5, 1850, 1350, 0.3)), // 5 feet
              entry(Units.inchesToMeters(91.6), new ShotParameter(-38, 2000, 1400, 0.23)), // 7 feet
              entry(
                  Units.inchesToMeters(114.2), new ShotParameter(-33.5, 2250, 1450, 0.2)), // 9 feet
              entry(
                  Units.inchesToMeters(136.5), new ShotParameter(-29, 2650, 1800, 0.175) // 11 feet
                  ),
              entry(
                  Units.inchesToMeters(160.9), new ShotParameter(-26.5, 2650, 1800, 0.15) // 13 feet
                  ),
              entry(
                  Units.inchesToMeters(182), new ShotParameter(-24.5, 2650, 1800, 0.125) // 15 feet
                  ),
              entry(
                  Units.inchesToMeters(215), new ShotParameter(-23.5, 2650, 1800, 0.125) // 17 feet
                  ),
              entry(
                  Units.inchesToMeters(226.7),
                  new ShotParameter(-21.75, 2700, 1850, 0.125) // 19 feet
                  ),
              entry(
                  Units.inchesToMeters(250), new ShotParameter(-21, 2750, 1900, 0.125) // 21 feet
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
