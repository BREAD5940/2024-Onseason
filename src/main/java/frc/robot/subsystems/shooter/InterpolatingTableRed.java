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
              entry(Units.inchesToMeters(50.2), new ShotParameter(-50, 2800, 2000, 0.2)), // 3 feet
              entry(
                  Units.inchesToMeters(70.3600640440825),
                  new ShotParameter(-42, 2800, 2000, 0.15)), // 5 feet
              entry(
                  Units.inchesToMeters(93.19887315858809),
                  new ShotParameter(-34, 2800, 2000, 0.05)), // 7 feet
              entry(
                  Units.inchesToMeters(115.63474279582495),
                  new ShotParameter(-28.5, 2800, 2000, 0.015)), // 9 feet
              entry(
                  Units.inchesToMeters(144.74300744842165),
                  new ShotParameter(-25.5, 2800, 2000, 0.015)), // 11 feet
              entry(
                  Units.inchesToMeters(159.77018646646147),
                  new ShotParameter(-23.5, 2800, 2000, 0.015)), // 13 feet
              entry(
                  Units.inchesToMeters(188.42587105703836),
                  new ShotParameter(-21, 2800, 2000, 0.015)), // 15 feet
              entry(
                  Units.inchesToMeters(211.29391027830243),
                  new ShotParameter(-19.5, 2800, 2000, 0.015)), // 17 feet
              entry(
                  Units.inchesToMeters(236.76671547817494),
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
