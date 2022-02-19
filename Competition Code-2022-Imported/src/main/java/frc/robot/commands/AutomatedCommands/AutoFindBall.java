package frc.robot.commands.AutomatedCommands;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoFindBall extends SequentialCommandGroup {
  private Vision vision;
  private SwerveRotaters rotaters;
  private SwerveSpinners spinners;
  private Gyro gyro;
  private Intake intake;

  public AutoFindBall(
      Vision vision, SwerveRotaters rotaters, SwerveSpinners spinners, Gyro gyro, Intake intake) {
    this.vision = vision;
    this.rotaters = rotaters;
    this.spinners = spinners;
    this.gyro = gyro;
    this.intake = intake;

    addCommands(
        new AutoVisionRotate(vision, rotaters, spinners, gyro),
        new AutoVisionDriveAndIntake(vision, rotaters, spinners, gyro, intake));
  }
}
