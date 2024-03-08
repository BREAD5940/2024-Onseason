package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTableBlue {

  // 10 and 1/8 inches is the distance
  // 83 inches and 1/2 to

  private InterpolatingTableBlue() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(

              // SFR BLUE
              entry(
                  Units.inchesToMeters(53.1),
                  new ShotParameter(-54.0, 1700, 1200, 0.33)), // (first) 3 feet
              entry(
                  Units.inchesToMeters(76.3),
                  new ShotParameter(-44.5, 1850.000000, 1350.000000, 0.3)), // 5 feet
              entry(
                  Units.inchesToMeters(99.6),
                  new ShotParameter(-39.0, 2000.000000, 1400.000000, 0.23)), // 7 feet
              entry(
                  Units.inchesToMeters(123.2),
                  new ShotParameter(-33.500000, 2250.000000, 1450.000000, 0.2)), // 9 feet
              entry(
                  Units.inchesToMeters(146.7),
                  new ShotParameter(-30.000000, 2650.000000, 1800.000000, 0.175)), // 11 feet
              entry(
                  Units.inchesToMeters(170.0),
                  new ShotParameter(-28.000000, 2650.000000, 1800.000000, 0.15)), // 13 feet
              entry(
                  Units.inchesToMeters(194),
                  new ShotParameter(-26.700000, 2650.000000, 1800.000000, 0.125)), // 15 feet
              entry(
                  Units.inchesToMeters(220),
                  new ShotParameter(-23.000000, 2650.000000, 1800.000000, 0.125)), // 17 feet
              entry(
                  Units.inchesToMeters(244),
                  new ShotParameter(-22.300000, 2700.000000, 1850.000000, 0.125)), // 19 feet
              entry(
                  Units.inchesToMeters(260),
                  new ShotParameter(-21.400000, 2750.000000, 1900.000000, 0.125)) // 21 feet
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
