// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class MoveForward extends CommandBase {
  private long startTime;
  private final Drive DRIVE;
  private final Double SPEED;
  private final Double DISTANCE;

  /** Creates a new MoveForward. */
  public MoveForward(Drive drive, Double distance, Double speed) {
    this.DRIVE = drive;
    this.DISTANCE = distance;
    this.SPEED = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVE.drive(new ChassisSpeeds(0, SPEED, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - DISTANCE / SPEED * 1000 > startTime);
  }
}
