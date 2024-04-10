package frc.robot.constants;

import java.util.HashMap;

public class SingleTagAdjusters {

  private SingleTagAdjusters() {}

  private static HashMap<Integer, Double> SingleTagAdjustersTable = new HashMap<Integer, Double>();

  static {
    SingleTagAdjustersTable.put(1, 1.0);
    SingleTagAdjustersTable.put(2, 1.0);
    SingleTagAdjustersTable.put(3, 1.0);
    SingleTagAdjustersTable.put(4, 1.0);
    SingleTagAdjustersTable.put(5, 1.0);
    SingleTagAdjustersTable.put(6, 1.0);
    SingleTagAdjustersTable.put(7, 1.0);
    SingleTagAdjustersTable.put(8, 1.0);
    SingleTagAdjustersTable.put(9, 1.0);
    SingleTagAdjustersTable.put(10, 1.0);
    SingleTagAdjustersTable.put(11, 1.0);
    SingleTagAdjustersTable.put(12, 1.0);
    SingleTagAdjustersTable.put(13, 1.0);
    SingleTagAdjustersTable.put(14, 1.0);
    SingleTagAdjustersTable.put(15, 1.0);
    SingleTagAdjustersTable.put(16, 1.0);
  }

  public static double getAdjustmentForTag(int tagId) {
    return SingleTagAdjustersTable.get(tagId);
  }
}
