// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

public class AutoAlign extends CommandBase {
  SwerveRotaters rotators;
  SwerveSpinners spinners;
  double targetAngle, fR, fL, bR, bL;
  Pixy pixy;

  public AutoAlign(SwerveRotaters rotators, SwerveSpinners spinners, Pixy pixy) {
    this.rotators = rotators;
    this.spinners = spinners;
    this.pixy = pixy;
    this.fR = rotators.angleToPulse(45);
    this.fL = rotators.angleToPulse(135);
    this.bL = rotators.angleToPulse(225);
    this.bR = rotators.angleToPulse(315);
    addRequirements(rotators, spinners, pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotators.setWheelDirection(fR, fL, bR, bL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotators.setWheelDirection(fR, fL, bR, bL);
    if (rotators.reachedPosition(fR, fL, bR, bL)) {
      spinners.runSpinners(AUTO_ALIGN_SPEED * pixy.getTurnDirection());
    } else {
      spinners.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotators.stop();
    spinners.stop();
    System.out.println("Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pixy.reachedTarget();
  }
}
