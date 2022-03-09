/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* ====================MAIN============================*/
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

public class RobotContainer {

  // == JOYSTICKS == //

  public final Joystick shopper = new Joystick(DRIVER_CONTROLLER);
  final Joystick operator = new Joystick(OPERATOR_CONTROLLER);

  // == SUBSYSTEMS == //

  // Drivetrain Subs
  private final Drive m_drivetrainSubsystem = new Drive();

  // Mechanism Subs
  public final Catapult CATAPULT = new Catapult();

  public final Intake INTAKE = new Intake();
  public final Climber CLIMBER = new Climber();
  // public final CompressorF COMPRESSOR = new CompressorF();

  public Intake getIntake() {
    return INTAKE;
  }

  public Climber getClimber() {
    return CLIMBER;
  }

  public Catapult getCatapult() {
    return CATAPULT;
  }

  public Drive getDrive() {
    return m_drivetrainSubsystem;
  }

  // == BUTTONS == //

  // Intake
  public final JoystickButton intakeButton = new JoystickButton(operator, INTAKE_BUTTON);
  public final JoystickButton raiseIntakeButton = new JoystickButton(operator, RAISE_INTAKE_BUTTON);

  // Catapult

  public final JoystickButton lowerCatapultButton =
      new JoystickButton(operator, LOWERCATAPULT_BUTTON);
  public final JoystickButton releaseCatapultButton =
      new JoystickButton(operator, RELEASECATAPULT_BUTTON);

  // == COMMANDS == //

  // Intake Commands

  public final Command intakeCommand = new IntakeCommand(INTAKE);
  public final Command raiseIntakeCommand = new RaiseIntakeCommand(INTAKE);
  // Catapult Commands
  public final Command releaseCatapultCommand = new ReleaseCatapultCommand(CATAPULT);
  public final Command lowerCatapultCommand = new LowerCatapultCommand(CATAPULT);

  public final Command extendCommand = new PowerTelescopingCommand(CLIMBER, INTAKE, -1);
  public final Command retractCommand = new PowerTelescopingCommand(CLIMBER, INTAKE, 1);

  // This constructs the robot container class.
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(
        new DriveCommand(
            m_drivetrainSubsystem,
            () ->
                -modifyAxis(shopper.getRawAxis(TRANSLATIONAL_VERTICAL_AXIS))
                    * Drive.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(shopper.getRawAxis(TRANSLATIONAL_HORIZONTAL_AXIS))
                    * Drive.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(shopper.getRawAxis(ROTATIONAL_HORIZONTAL_AXIS))
                    * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    CLIMBER.setDefaultCommand(
        new RunCommand(
            () ->
                CLIMBER.supplyTelescoping(
                    operator.getRawAxis(OPERATOR_L3),
                    operator.getRawAxis(OPERATOR_R3))));

    INTAKE.setDefaultCommand(new RunCommand(() -> INTAKE.intake(), INTAKE));
    // Catapult
    lowerCatapultButton.whenHeld(lowerCatapultCommand);
    releaseCatapultButton.whenPressed(releaseCatapultCommand);

    // Intake
    intakeButton.whileHeld(intakeCommand);
    raiseIntakeButton.whenPressed(raiseIntakeCommand);

    // Climber
    // climbButton.whenHeld(climbSequence);
    // extend.whenHeld(extendCommand);
    // retract.whenHeld(retractCommand);
    // stay.whenHeld(stayCommand);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
