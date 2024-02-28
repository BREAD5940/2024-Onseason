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
              // FIRST
              // entry(1.3301072004135244, new ShotParameter(-45, 2000, 4000, 0.3)),
              // entry(1.905, new ShotParameter(-42, 2000, 4000, 0.250000)),
              // entry(2.461135414571067, new ShotParameter(-32, 2000, 4000, 0.22)),
              // entry(3.169917691840288, new ShotParameter(-30, 2000, 4000, 0.2)),
              // entry(3.7826524711538756, new ShotParameter(-26, 2000, 4000, 0.18)),
              // entry(4.121620753510814, new ShotParameter(-26, 2000, 4000, 0.16)),
              // entry(4.544767156900149, new ShotParameter(-25.000000, 2000, 4000, 0.15)),
              // entry(5.1824951858114305, new ShotParameter(-23.250000, 2400, 4400,
              // 0.125000)))entry(1.3301072004135244, new ShotParameter(-45, 2000, 4000, 0.3)),
              // SECOND
              // entry(Units.inchesToMeters(79.6), new ShotParameter(-40, 4000, 2000, 0.3)),
              // entry(Units.inchesToMeters(105.8), new ShotParameter(-34, 4000, 2000, 0.25)),
              // entry(Units.inchesToMeters(130.1), new ShotParameter(-28.5, 4100, 2100, 0.25)),
              // entry(Units.inchesToMeters(155.4), new ShotParameter(-27, 4100, 2100, 0.175)),
              // entry(Units.inchesToMeters(180.5), new ShotParameter(-25, 4100, 2100, 0.175)),
              // entry(Units.inchesToMeters(203.4), new ShotParameter(-23.5, 4200, 2200, 0.16)),
              // entry(Units.inchesToMeters(225.7), new ShotParameter(-22, 4600, 2600, 0.125))));
              // THIRD
              entry(1.2629021804364189, new ShotParameter(-50.0, 2000, 1500, 0.4)),
              entry(1.8709308362652906, new ShotParameter(-40.0, 2000, 1500, 0.3)),
              entry(2.4528234316913737, new ShotParameter(-35.0, 2400, 1800, 0.23)),
              entry(3.0075105927719874, new ShotParameter(-31.0, 2400, 1800, 0.2)),
              entry(3.570768447818622, new ShotParameter(-28.0, 2400, 1800, 0.175)),
              entry(4.141237376169209, new ShotParameter(-24.0, 2666, 2000, 0.15)),
              entry(4.779837214305064, new ShotParameter(-22.0, 3000, 2000, 0.125)),
              entry(5.4, new ShotParameter(-21.0, 3000, 2000, 0.125)),
              entry(5.95, new ShotParameter(-20.0, 3300, 2200, 0.125)),
              entry(6.55, new ShotParameter(-18.0, 2600, 2200, 0.125))));

  // limp -50 1800 1800 0.4

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
