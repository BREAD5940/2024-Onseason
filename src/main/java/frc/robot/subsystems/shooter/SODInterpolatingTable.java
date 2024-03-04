package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class SODInterpolatingTable {

  private SODInterpolatingTable() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(2.0216634299077194, new ShotParameter(-29, 3000, 1500, 0.6)),
              entry(2.553, new ShotParameter(-26.0, 3000, 1500, 0.6)),
              entry(3.9052370831553973, new ShotParameter(-17.5, 3000, 1500, 0.6))));

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
