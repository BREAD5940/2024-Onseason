package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class SODInterpolatingTableBlue {

  private SODInterpolatingTableBlue() {}

  public static TreeMap<Double, ShotParameter> table =
      new TreeMap<>(
          Map.ofEntries(
              entry(Units.inchesToMeters(115.7), new ShotParameter(-22, 2500, 1500, 0.6)),
              entry(Units.inchesToMeters(136.0), new ShotParameter(-18.5, 2500, 1500, 0.6))));

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
