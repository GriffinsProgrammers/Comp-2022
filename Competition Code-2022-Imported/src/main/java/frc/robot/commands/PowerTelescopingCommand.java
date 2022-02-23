// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

import static frc.robot.Constants.*;

public class PowerTelescopingCommand extends CommandBase {
  // Creates a new Command for extending Telescoping/Primary Climbing Arms
  private Climber CLIMBER;
  private Intake INTAKE;
  private double power;

  public PowerTelescopingCommand(Climber CLIMBER, Intake INTAKE, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CLIMBER);
    this.CLIMBER = CLIMBER;
    this.INTAKE = INTAKE;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (INTAKE.getPistonState()) {
    // This means that 1 second after the command is initialized, the command will end.
    // CLIMBER.supplyTelescoping(power);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // CLIMBER.supplyTelescoping(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
