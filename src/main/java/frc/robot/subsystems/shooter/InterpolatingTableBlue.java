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
                  Units.inchesToMeters(52.98391497884648),
                  new ShotParameter(-50, 2800, 2000, 0.2)), // 3 feet
              entry(
                  Units.inchesToMeters(75.58317704313004),
                  new ShotParameter(-42, 2800, 2000, 0.15)), // 5 feet
              entry(
                  Units.inchesToMeters(98.6221813946583),
                  new ShotParameter(-34, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(120.8518443837398),
                  new ShotParameter(-28.5, 2800, 2000, 0.015)), // 9 feet
              entry(
                  Units.inchesToMeters(145.88379967304252),
                  new ShotParameter(-25.5, 2800, 2000, 0.015)), // 11 feet
              entry(
                  Units.inchesToMeters(172.8301935879358),
                  new ShotParameter(-23.5, 2800, 2000, 0.015)), // 13 feet
              entry(
                  Units.inchesToMeters(197.2502943739088),
                  new ShotParameter(-21, 2800, 2000, 0.015)), // 15 feet
              entry(
                  Units.inchesToMeters(220.37003190905122),
                  new ShotParameter(-19.5, 2800, 2000, 0.015)), // 17 feet
              entry(
                  Units.inchesToMeters(244.48903752524166),
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
