package frc.robot.vision.northstar;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  public static class AprilTagVisionIOInputs implements LoggableInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
    public long version = 2;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", frames.length);
      for (int i = 0; i < frames.length; i++) {
        table.put("Frame/" + Integer.toString(i), frames[i]);
      }
      table.put("Fps", fps);
      table.put("Version", version);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", timestamps);
      int frameCount = (int) table.get("FrameCount", 0);
      frames = new double[frameCount][];
      for (int i = 0; i < frameCount; i++) {
        frames[i] = table.get("Frame/" + Integer.toString(i), new double[] {});
      }
      fps = table.get("Fps", fps);
      version = table.get("Version", version);
    }
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
