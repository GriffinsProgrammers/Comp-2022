// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoMove extends CommandBase {

  SwerveRotaters rotators;
  SwerveSpinners spinners;
  double pulsesDistance;
  Gyro gyro;
  double fR, fL, bR, bL;

  // This command sets the wheels to a specific angle and drives the robot a certain
  // distance in that direction.
  public AutoMove(
      SwerveRotaters rotators,
      SwerveSpinners spinners,
      Gyro gyro,
      double angle,
      double moveDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotators = rotators;
    this.spinners = spinners;
    this.gyro = gyro;
    this.fR = rotators.angleToPulse(angle, gyro.getYaw());
    this.fL = rotators.angleToPulse(angle, gyro.getYaw());
    this.bR = rotators.angleToPulse(angle, gyro.getYaw());
    this.bL = rotators.angleToPulse(angle, gyro.getYaw());
    addRequirements(rotators, spinners);
    this.pulsesDistance = spinners.cmToPulses(moveDistance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AutoMove Initialized");
    spinners.resetEncoders();
    rotators.setWheelDirection(fR, fL, bR, bL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotators.setWheelDirection(fR, fL, bR, bL);
    if (rotators.reachedPosition(fR, fL, bR, bL)) {
      spinners.driveDistance(pulsesDistance);
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
    return spinners.reachedPosition(pulsesDistance);
  }
}
