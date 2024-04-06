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
              // entry(
              //     Units.inchesToMeters(52.75),
              //     new ShotParameter(
              //         -54.0,
              //         1700,
              //         1200,
              //         0.33 - Units.inchesToMeters(5.0))), // (first) 3 feet - 54in.
              // entry(
              //     Units.inchesToMeters(72.8),
              //     new ShotParameter(
              //         -44.5, 1850, 1350, 0.3 - Units.inchesToMeters(5.0))), // 5 feet - 78in
              // entry(
              //     Units.inchesToMeters(96.5),
              //     new ShotParameter(
              //         -38, 2000, 1400, 0.23 - Units.inchesToMeters(5.0))), // 7 feet - 102in
              // entry(
              //     Units.inchesToMeters(120.0),
              //     new ShotParameter(
              //         -33.5, 2250, 1450, 0.2 - Units.inchesToMeters(5.0))), // 9 feet - 126in.
              // entry(
              //     Units.inchesToMeters(145.5),
              //     new ShotParameter(
              //         -29, 2650, 1800, 0.175 - Units.inchesToMeters(5.0)) // 11 feet - 150in.
              //     ),
              // entry(
              //     Units.inchesToMeters(171),
              //     new ShotParameter(
              //         -26.5, 2650, 1800, 0.15 - Units.inchesToMeters(5.0)) // 13 feet - 174in.
              //     ),
              // entry(
              //     Units.inchesToMeters(193),
              //     new ShotParameter(
              //         -24.5, 2650, 1800, 0.125 - Units.inchesToMeters(5.0)) // 15 feet - 198in.
              //     ),
              // entry(
              //     Units.inchesToMeters(219),
              //     new ShotParameter(
              //         -23.5, 2650, 1800, 0.125 - Units.inchesToMeters(5.0)) // 17 feet - 222in.
              //     ),
              // entry(
              //     Units.inchesToMeters(238),
              //     new ShotParameter(
              //         -21.75, 2700, 1850, 0.125 - Units.inchesToMeters(5.0)) // 19 feet - 246in.
              //     ),
              // entry(
              //     Units.inchesToMeters(266),
              //     new ShotParameter(
              //         -21, 2750, 1900, 0.125 - Units.inchesToMeters(5.0)) // 21 feet - 270in.
              // )
              entry(
                  Units.inchesToMeters(46.5), new ShotParameter(-50, 2800, 2000, 0.175)), // 3 feet
              entry(
                  Units.inchesToMeters(70.0), new ShotParameter(-42, 2800, 2000, 0.125)), // 5 feet
              entry(
                  Units.inchesToMeters(95.25),
                  new ShotParameter(-34.75, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(118.0), new ShotParameter(-30, 2800, 2000, 0.05)), // 9 feet
              entry(
                  Units.inchesToMeters(142.0), new ShotParameter(-28, 2800, 2000, 0.0)), // 11 feet
              entry(
                  Units.inchesToMeters(168.25),
                  new ShotParameter(-24.75, 2800, 2000, 0.0)), // 13 feet
              entry(
                  Units.inchesToMeters(194), new ShotParameter(-23.75, 2800, 2000, 0.0)), // 15 feet
              entry(
                  Units.inchesToMeters(215), new ShotParameter(-21.9, 2800, 2000, 0.0)), // 17 feet
              entry(
                  Units.inchesToMeters(239.0), new ShotParameter(-21, 3200, 2100, 0.0)), // 19 feet
              entry(
                  Units.inchesToMeters(262.5), new ShotParameter(-20, 3200, 2100, 0.0) // 21 feet
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
