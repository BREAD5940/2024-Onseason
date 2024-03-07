package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.GOAL_INWARD_SHIFT;
import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTable {

  // 10 and 1/8 inches is the distance
  // 83 inches and 1/2 to

  private InterpolatingTable() {}

  private static final double EDGE_TO_CENTER = Units.inchesToMeters(18);

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
              // entry(1.2629021804364189, new ShotParameter(-51.0, 2000, 1500, 0.4)),
              // entry(1.8709308362652906, new ShotParameter(-43.0, 2000, 1500, 0.3)),
              // entry(2.4528234316913737, new ShotParameter(-39.0, 2400, 1800, 0.23)),
              // entry(3.0075105927719874, new ShotParameter(-33.0, 2400, 1800, 0.2)),
              // entry(3.570768447818622, new ShotParameter(-30.0, 2400, 1800, 0.175)),
              // entry(4.141237376169209, new ShotParameter(-27.0, 2666, 2000, 0.15)),
              // entry(4.779837214305064, new ShotParameter(-25.0, 3000, 2000, 0.125)),
              // entry(5.4, new ShotParameter(-21.0, 3000, 2000, 0.125)),
              // entry(5.95, new ShotParameter(-21.0, 3300, 2200, 0.125)),
              //             entry(6.55, new ShotParameter(-19.0, 2600, 2200, 0.125))));
              // FIFTH
              // entry(
              //     0.9533967832127577 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-54.0, 1700, 1200, 0.33)), // (first) 0 feet
              // entry(
              //     1.5379214993123282 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-44.5, 1850.000000, 1350.000000, 0.3)), // 5 feet
              // entry(
              //     2.135834451111118 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-39.0, 2000.000000, 1400.000000, 0.23)), // 7 feet
              // entry(
              //     2.713838083504854 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-33.500000, 2250.000000, 1450.000000, 0.2)), // 9 feet
              // entry(
              //     3.314519093402443 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-30.000000, 2650.000000, 1800.000000, 0.175)), // 11 feet
              // entry(
              //     3.884010466202368 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-28.000000, 2650.000000, 1800.000000, 0.15)), // 13 feet
              // entry(
              //     4.391338677115708 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-26.700000, 2650.000000, 1800.000000, 0.125)), // 15 feet
              // entry(
              //     5.117897012633007 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-23.000000, 2650.000000, 1800.000000, 0.125)), // 17 feet
              // entry(
              //     5.6431411860907055 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-22.300000, 2700.000000, 1850.000000, 0.125)), // 19 feet
              // entry(
              //     6.159945658209746 + INWARD_SHIFT_AMOUNT,
              //     new ShotParameter(-21.400000, 2750.000000, 1900.000000, 0.125)))); // 21 feet

              // 60 inches
              // SIXTH
              entry(
                  Units.feetToMeters(3) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-54.0, 1700, 1200, 0.33)), // (first) 3 feet
              entry(
                  Units.feetToMeters(5) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-44.5, 1850.000000, 1350.000000, 0.3)), // 5 feet
              entry(
                  Units.feetToMeters(7) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-39.0, 2000.000000, 1400.000000, 0.23)), // 7 feet
              entry(
                  Units.feetToMeters(9) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-33.500000, 2250.000000, 1450.000000, 0.2)), // 9 feet
              entry(
                  Units.feetToMeters(11) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-30.000000, 2650.000000, 1800.000000, 0.175)), // 11 feet
              entry(
                  Units.feetToMeters(13) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-28.000000, 2650.000000, 1800.000000, 0.15)), // 13 feet
              entry(
                  Units.feetToMeters(15) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-26.700000, 2650.000000, 1800.000000, 0.125)), // 15 feet
              entry(
                  Units.feetToMeters(17) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-23.000000, 2650.000000, 1800.000000, 0.125)), // 17 feet
              entry(
                  Units.feetToMeters(19) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-22.300000, 2700.000000, 1850.000000, 0.125)), // 19 feet
              entry(
                  Units.feetToMeters(21) + EDGE_TO_CENTER + GOAL_INWARD_SHIFT,
                  new ShotParameter(-21.400000, 2750.000000, 1900.000000, 0.125)))); // 21 feet

  // FOURTH
  // entry(1.15, new ShotParameter(-51.0, 2000, 1500, 0.4)),
  // entry(1.66, new ShotParameter(-45.0, 2500, 1500, 0.3)),
  // entry(2.29, new ShotParameter(-39.0, 2500, 1500, 0.23)),
  // entry(2.815, new ShotParameter(-33.0, 3000, 1800, 0.2)),
  // entry(3.35, new ShotParameter(-30.0, 3200, 2200, 0.175)),
  // entry(3.86, new ShotParameter(-25.5, 3200, 2200, 0.15)),
  // entry(4.65, new ShotParameter(-24, 3500, 2300, 0.125)),
  // entry(5.05, new ShotParameter(-22, 3600, 2300, 0.125)),
  // entry(5.65, new ShotParameter(-20.5, 3800, 2400, 0.125)),
  // entry(6.5, new ShotParameter(-19, 3900, 2500, 0.125))));

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
