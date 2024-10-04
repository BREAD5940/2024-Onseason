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
                  Units.inchesToMeters(51.190304936947676),
                  new ShotParameter(-50, 2800, 2000, 0.2)), // 3 feet
              entry(
                  Units.inchesToMeters(71.79757741208864),
                  new ShotParameter(-42, 2800, 2000, 0.15)), // 5 feet
              entry(
                  Units.inchesToMeters(95.17512142976624),
                  new ShotParameter(-34, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(119.61357436402967),
                  new ShotParameter(-28.5, 2800, 2000, 0.015)), // 9 feet
              entry(
                  Units.inchesToMeters(145.4206472677241),
                  new ShotParameter(-25.5, 2800, 2000, 0.015)), // 11 feet
              entry(
                  Units.inchesToMeters(167.41538306777346), // 10 mayve check
                  new ShotParameter(-23.5, 2800, 2000, 0.015)), // 13 feet
              entry(
                  Units.inchesToMeters(192.10640639325774),
                  new ShotParameter(-21, 2800, 2000, 0.015)), // 15 feet
              entry(
                  Units.inchesToMeters(216.67763866698337),
                  new ShotParameter(-19.5, 2800, 2000, 0.015)), // 17 feet
              entry(
                  Units.inchesToMeters(240.2850210919894), // 16
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
