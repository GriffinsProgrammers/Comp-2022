package frc.robot.commands.AutomatedCommands;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoVisionDrive extends CommandBase {
  private Vision vision;
  private SwerveRotaters rotaters;
  private SwerveSpinners spinners;
  private Gyro gyro;
  private double fR, fL, bR, bL;

  public AutoVisionDrive(
      Vision vision, SwerveRotaters rotaters, SwerveSpinners spinners, Gyro gyro) {
    this.vision = vision;
    this.rotaters = rotaters;
    this.spinners = spinners;
    this.gyro = gyro;
    this.fR = 0;
    this.fL = 0;
    this.bR = 0;
    this.bL = 0;
    addRequirements(rotaters, spinners);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotaters.setWheelDirection(fR, fL, bR, bL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotaters.setWheelDirection(fR, fL, bR, bL);
    if (rotaters.reachedPosition(fR, fL, bR, bL)) {
      spinners.driveDistance(2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotaters.stop();
    spinners.stop();
    System.out.println("Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getArea() > 80.0;
  }
}
