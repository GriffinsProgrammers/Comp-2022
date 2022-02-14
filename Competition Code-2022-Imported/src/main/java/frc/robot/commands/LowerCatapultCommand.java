// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Catapult;
import static frc.robot.Constants.*;

public class LowerCatapultCommand extends CommandBase {
  // Creates wind catapult arm command
  private Catapult CATAPULT;
  private DoubleSolenoid.Value shooter_piston_state;
  private boolean piston_retracted = false;
  private double retractTimer;

  public LowerCatapultCommand(Catapult CATAPULT) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CATAPULT);

    this.CATAPULT = CATAPULT;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter_piston_state = CATAPULT.getPistonState();
    if (shooter_piston_state == DoubleSolenoid.Value.kOff
        || shooter_piston_state == DoubleSolenoid.Value.kForward) {
      CATAPULT.retractPiston();
      piston_retracted = true;
      retractTimer = System.currentTimeMillis();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (piston_retracted && System.currentTimeMillis() - retractTimer > 1000) {
      CATAPULT.setSpeed();
    } else if (!piston_retracted) {
      CATAPULT.setSpeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CATAPULT.motorsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
