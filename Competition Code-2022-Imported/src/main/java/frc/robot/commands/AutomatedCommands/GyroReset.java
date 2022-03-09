// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Gyro;

public class GyroReset extends CommandBase {

  private Gyro gyro;

  /** Creates a new GyroReset. */
  public GyroReset(Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gyro);
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.resetValues();
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
    return false;
  }
}
