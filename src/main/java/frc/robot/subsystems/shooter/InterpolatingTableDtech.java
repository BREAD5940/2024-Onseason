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
                  Units.inchesToMeters(52.75),
                  new ShotParameter(-54.0, 1700, 1200, 0.33)), // (first) 3 feet - 54in.
              entry(
                  Units.inchesToMeters(72.8),
                  new ShotParameter(-44.5, 1850, 1350, 0.3)), // 5 feet - 78in
              entry(
                  Units.inchesToMeters(96.5),
                  new ShotParameter(-38, 2000, 1400, 0.23)), // 7 feet - 102in
              entry(
                  Units.inchesToMeters(120.0),
                  new ShotParameter(-33.5, 2250, 1450, 0.2)), // 9 feet - 126in.
              entry(
                  Units.inchesToMeters(145.5),
                  new ShotParameter(-29, 2650, 1800, 0.175) // 11 feet - 150in.
                  ),
              entry(
                  Units.inchesToMeters(171),
                  new ShotParameter(-26.5, 2650, 1800, 0.15) // 13 feet - 174in.
                  ),
              entry(
                  Units.inchesToMeters(193),
                  new ShotParameter(-24.5, 2650, 1800, 0.125) // 15 feet - 198in.
                  ),
              entry(
                  Units.inchesToMeters(219),
                  new ShotParameter(-23.5, 2650, 1800, 0.125) // 17 feet - 222in.
                  ),
              entry(
                  Units.inchesToMeters(238),
                  new ShotParameter(-21.75, 2700, 1850, 0.125) // 19 feet - 246in.
                  ),
              entry(
                  Units.inchesToMeters(266),
                  new ShotParameter(-21, 2750, 1900, 0.125) // 21 feet - 270in.
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
