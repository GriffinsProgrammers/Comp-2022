// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PowerTelescopingCommand;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new autonomous command sequence. */
  public Climber climber;

  private double T1, T2, T3, T4, T5, T6, T7;

  /**
   * @param climber
   * @param rotators
   * @param spinners
   */
  public ClimbSequence(Climber climber, SwerveRotaters rotators, SwerveSpinners spinners) {
    // This is the sequential commands for our climbing sequence
    this.climber = climber;
    T1 = 1.0;
    T2 = 1.0;
    T3 = 1.0;
    T4 = 1.0;
    T5 = 1.0;
    T6 = 1.0;
    T7 = 1.0;

    addCommands(
        // The telescoping is initialized
        new PowerTelescopingCommand(climber, TELESCOPING_INITIALIZE),
        new WaitCommand(T1),
        // The telescopings are let to max height
        new PowerTelescopingCommand(climber, 0),
        new WaitCommand(T2),
        //  The robot moves into position
        new MoveForward(rotators, spinners, T3),
        new WaitCommand(T4),
        // telescopings latch onto bar
        new PowerTelescopingCommand(climber, LOWPOWER),
        new WaitCommand(T5),
        // more power to telescoping to lift robot
        new PowerTelescopingCommand(climber, HIGHPOWER),
        new WaitCommand(T6),
        // power set to keep the robot where it is
        new PowerTelescopingCommand(climber, MIDPOWER),
        new WaitCommand(T7));

    // Is this necessary? TODO: Review this.
  }
}
