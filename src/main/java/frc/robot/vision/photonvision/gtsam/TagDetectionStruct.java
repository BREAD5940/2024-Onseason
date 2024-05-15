package frc.robot.vision.photonvision.gtsam;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.List;
import org.photonvision.targeting.TargetCorner;

public class TagDetectionStruct implements Struct<TagDetection> {
  @Override
  public Class<TagDetection> getTypeClass() {
    return TagDetection.class;
  }

  @Override
  public String getTypeString() {
    return "struct:TagDetection";
  }

  @Override
  public int getSize() {
    return ((8 * 2) * 4 + 4);
  }

  @Override
  public String getSchema() {
    return "uint32 id;double cx1;double cy1;double cx2;double cy2;double "
        + "cx3;double cy3;double cx4;double cy4";
  }

  @Override
  public TagDetection unpack(ByteBuffer bb) {
    return new TagDetection(
        bb.getInt(),
        List.of(
            new TargetCorner(bb.getDouble(), bb.getDouble()),
            new TargetCorner(bb.getDouble(), bb.getDouble()),
            new TargetCorner(bb.getDouble(), bb.getDouble()),
            new TargetCorner(bb.getDouble(), bb.getDouble())));
  }

  @Override
  public void pack(ByteBuffer bb, TagDetection value) {
    bb.putInt(value.id);
    for (int i = 0; i < 4; i++) {
      bb.putDouble(value.corners.get(i).x);
      bb.putDouble(value.corners.get(i).y);
    }
  }
}
