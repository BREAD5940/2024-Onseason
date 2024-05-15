package frc.robot.vision.photonvision.gtsam;

import java.util.List;
import org.photonvision.targeting.TargetCorner;

public class TagDetection {
  public final int id;
  public final List<TargetCorner> corners;

  public TagDetection(int id, List<TargetCorner> corners) {
    this.id = id;
    this.corners = corners;
  }

  public static final TagDetectionStruct struct = new TagDetectionStruct();
}
