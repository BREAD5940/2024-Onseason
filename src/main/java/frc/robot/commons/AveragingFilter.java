package frc.robot.commons;

import java.util.ArrayList;

public class AveragingFilter {

  private ArrayList<Double> values = new ArrayList<Double>();
  private int maxSize;

  public AveragingFilter(int maxSize) {
    this.maxSize = maxSize;
  }

  public double getAverage() {
    double sum = 0.0;
    for (Double value : values) {
      sum += value;
    }
    return (double) sum / values.size();
  }

  public void addSample(double sample) {
    values.add(sample);
    if (values.size() > maxSize) {
      values.remove(0);
    }
  }

  public int getSize() {
    return values.size();
  }

  public int getWindowSize() {
    return maxSize;
  }

  public void clear() {
    values.clear();
  }
}
