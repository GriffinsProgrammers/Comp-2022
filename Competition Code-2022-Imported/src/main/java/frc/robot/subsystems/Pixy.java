package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import static frc.robot.Constants.*;

public class Pixy extends SubsystemBase {

  Pixy2 pixy;
  double width;

  public Pixy() {
    pixy = Pixy2.createInstance(Pixy2.LinkType.SPI);
    pixy.init();
    width = pixy.getFrameWidth();
  }

  public double getTurnDirection() {
    double xCoordinate =
        getLargestBlock()
            .getX(); // This method should give the x coordinate of the center of the block
    return xCoordinate >= width / 2 ? -1 : 1;
  }

  public Block getLargestBlock() {
    ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
    if (blocks.size() > 0) {
      Block largestBlock = blocks.get(0);
      Block currentBlock;
      for (int i = 0; i < blocks.size(); i++) {
        currentBlock = blocks.get(i);
        if ((currentBlock.getWidth() * currentBlock.getHeight())
            > (largestBlock.getWidth() * largestBlock.getHeight())) {
          largestBlock = blocks.get(i);
        }
      }
      return largestBlock;
    }
    return null;
  }

  // TODO: Add code for auto positioning

  public boolean reachedTarget() {
    if (Math.abs(getLargestBlock().getX() - (width / 2)) <= AUTO_ALIGN_ERROR) return true;
    return false;
  }
}
