// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

public class SetRotatorsfortELEOP extends CommandBase {

  SwerveRotaters rotators;
  SwerveSpinners spinners;
  private long startTimer;
  /** Creates a new SetRotatorsfortELEOP. */
  public SetRotatorsfortELEOP(SwerveRotaters rotaters) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotators = rotators;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotators.setWheelDirection(90, 90, 90, 90);
    startTimer = System.currentTimeMillis();
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

    return (System.currentTimeMillis() - 2000 > startTimer);
  }
}
