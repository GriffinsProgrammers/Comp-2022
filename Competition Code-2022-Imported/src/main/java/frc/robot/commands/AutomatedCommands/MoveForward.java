// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

public class MoveForward extends CommandBase {

  SwerveRotaters rotators;
  SwerveSpinners spinners;
  private double moveTime;
  private double startTime = 0;

  // This command sets the wheels to a specific angle and drives the robot a certain
  // distance in that direction.
  public MoveForward(SwerveRotaters rotators, SwerveSpinners spinners, double moveTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotators = rotators;
    this.spinners = spinners;
    this.moveTime = moveTime;
    addRequirements(rotators, spinners);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    System.out.println("MoveForward Initialized");
    spinners.resetEncoders();
    rotators.setWheelDirection(0, 0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotators.setWheelDirection(0, 0, 0, 0);
    if (rotators.reachedPosition(0, 0, 0, 0)) {
      spinners.runSpinners(MOVE_SPEED);
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
    return (System.currentTimeMillis() - startTime) > 1000 * moveTime;
  }
}
