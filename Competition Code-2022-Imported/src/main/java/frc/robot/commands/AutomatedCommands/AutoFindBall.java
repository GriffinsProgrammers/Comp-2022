package frc.robot.commands.AutomatedCommands;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoFindBall extends SequentialCommandGroup {
  private Vision vision;
  private SwerveRotaters rotaters;
  private SwerveSpinners spinners;
  private Gyro gyro;

  public AutoFindBall(Vision vision, SwerveRotaters rotaters, SwerveSpinners spinners, Gyro gyro) {
    this.vision = vision;
    this.rotaters = rotaters;
    this.spinners = spinners;
    this.gyro = gyro;

    addCommands(
        new AutoVisionRotate(vision, rotaters, spinners, gyro),
        new AutoVisionDrive(vision, rotaters, spinners, gyro));
  }
}
