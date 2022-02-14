// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;

public class ReleaseCatapultCommand extends CommandBase {
  // Creates release catapult for shooting command

  private Catapult CATAPULT;
  private Intake INTAKE;
  private double startTime;

  public ReleaseCatapultCommand(Catapult catapult, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(catapult);
    this.CATAPULT = catapult;

    addRequirements(intake);
    this.INTAKE = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Only shoot if the intake is lowered
    if (INTAKE.getPistonState()) {
      // For shooting we need to extend the piston.
      CATAPULT.extendPiston();
      startTime = System.currentTimeMillis();
    }
    // A variable is created for determining when we want to end command.

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When the command ends, the piston is returned to its original position.
    // Therefore, the mechanism is now ready for lowering the catapult once more.
    CATAPULT.retractPiston();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Only checks if the intake pistons are down, which means that the catapult has fired
    if (INTAKE.getPistonState()) {
      // This means that 1 second after the command is initialized, the command will end.
      return (System.currentTimeMillis() - startTime) > 1000;
    }
    return false;
  }
}
