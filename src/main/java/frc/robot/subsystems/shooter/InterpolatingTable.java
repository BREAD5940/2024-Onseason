package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTable {

  private InterpolatingTable() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(1.3301072004135244, new ShotParameter(-45, 2000, 4000, 0.3)),
              entry(1.905, new ShotParameter(-42, 2000, 4000, 0.250000)),
              entry(2.461135414571067, new ShotParameter(-32, 2000, 4000, 0.22)),
              entry(3.169917691840288, new ShotParameter(-30, 2000, 4000, 0.2)),
              entry(3.7826524711538756, new ShotParameter(-26, 2000, 4000, 0.18)),
              entry(4.121620753510814, new ShotParameter(-26, 2000, 4000, 0.16)),
              entry(4.544767156900149, new ShotParameter(-25.000000, 2000, 4000, 0.15)),
              entry(5.1824951858114305, new ShotParameter(-23.250000, 2400, 4400, 0.125000))));

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
