package frc.robot.commands.AutomatedCommands;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.commands.AutomatedCommands.*;
import static frc.robot.Constants.*;


public class AutoVisionRotate extends CommandBase{
    private Vision vision;
    private SwerveRotaters rotaters;
    private SwerveSpinners spinners;
    private Gyro gyro;
    private double targetAngle, fR, fL, bR, bL;
    private boolean gyroReady = false;

    public AutoVisionRotate(Vision vision,SwerveRotaters rotaters,
    SwerveSpinners spinners,Gyro gyro)
    {
        this.vision = vision;
        this.rotaters = rotaters;
        this.spinners = spinners;
        this.gyro = gyro;
        this.fR = rotaters.angleToPulse(45);
        this.fL = rotaters.angleToPulse(135);
        this.bL = rotaters.angleToPulse(225);
        this.bR = rotaters.angleToPulse(315);
        addRequirements(rotaters, spinners);
    }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (gyro.getGyroState() == 1) gyroReady = true;
    gyro.getState(); // Debugging
    rotaters.setWheelDirection(fR, fL, bR, bL);
    if (rotaters.reachedPosition(fR, fL, bR, bL) && gyroReady && vision.canSeeBall() && vision.getYaw()>0) 
    {
      spinners.runSpinners(AUTO_ROTATE_SPEED * 1);
    } 
    else if(rotaters.reachedPosition(fR, fL, bR, bL) && gyroReady && vision.canSeeBall() && vision.getYaw()<0)
    {
        spinners.runSpinners(AUTO_ROTATE_SPEED * -1);
    }
    else 
    {
      spinners.stop();
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
    return vision.canSeeBall()&&(vision.getYaw()<5||vision.getYaw()>-5);
  }
}
