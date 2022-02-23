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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.AutomatedCommands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

public class RobotContainer {
  private Command driveForward;
  private Command sixPoint;
  private Command tenPoint;
  private Command waitForTeleOp;
  private Command climbChild;

  // == JOYSTICKS == //

  public final Joystick shopper = new Joystick(DRIVER_CONTROLLER);
  final Joystick operator = new Joystick(OPERATOR_CONTROLLER);

  // == SUBSYSTEMS == //

  // Drivetrain Subs
  public final SwerveSpinners SWERVESPINNERS = new SwerveSpinners();
  public final SwerveRotaters SWERVEROTATERS = new SwerveRotaters();

  // Extra Subs
  public final Gyro GYRO = new Gyro();
  /*
    public final Pixy PIXY = new Pixy();
  */
  // Mechanism Subs
  public final Catapult CATAPULT = new Catapult();

  public final Intake INTAKE = new Intake();
  public final Climber CLIMBER = new Climber();
  // public final CompressorF COMPRESSOR = new CompressorF();

  // Auto Subs Access
  public SwerveSpinners getSpinners() {
    return SWERVESPINNERS;
  }

  public SwerveRotaters getRotaters() {
    return SWERVEROTATERS;
  }

  public Gyro getGyro() {
    return GYRO;
  }

  public Intake getIntake() {
    return INTAKE;
  }

  public Climber getClimber() {
    return CLIMBER;
  }

  public Catapult getCatapult() {
    return CATAPULT;
  }

  // == BUTTONS == //

  // Gyro
  public final JoystickButton gyroResetButton = new JoystickButton(operator, 8);

  // Intake
  public final JoystickButton intakeButton = new JoystickButton(operator, INTAKE_BUTTON);
  public final JoystickButton raiseIntakeButton = new JoystickButton(operator, RAISE_INTAKE_BUTTON);

  // Catapult

  public final JoystickButton lowerCatapultButton =
      new JoystickButton(operator, LOWERCATAPULT_BUTTON);
  public final JoystickButton releaseCatapultButton =
      new JoystickButton(operator, RELEASECATAPULT_BUTTON);
  // public final JoystickButton alignCatapultButton =
  //     new JoystickButton(operator, ALIGNCATAPULT_BUTTON);

  // Climber
  // public final JoystickButton climbButton = new JoystickButton(operator, CLIMB_BUTTON);
  public final JoystickButton extend = new JoystickButton(operator, BUTTON_Y);
  public final JoystickButton retract = new JoystickButton(operator, BUTTON_B);
  // public final JoystickButton stay = new JoystickButton(operator, BUTTON_Y);

  // == COMMANDS == //

  // Gyro Commands
  public final Command gyroResetCommand = new GyroReset(GYRO);

  // Intake Commands

  public final Command intakeCommand = new IntakeCommand(INTAKE);
  public final Command raiseIntakeCommand = new RaiseIntakeCommand(INTAKE);
  // Catapult Commands
  public final Command releaseCatapultCommand = new ReleaseCatapultCommand(CATAPULT);
  public final Command lowerCatapultCommand = new LowerCatapultCommand(CATAPULT);
  // public final Command alignCatapultCommand = new AutoAlign(SWERVEROTATERS, SWERVESPINNERS,
  // PIXY);
  // Climber Commands

  // public final Command climbSequence = new ClimbSequence(CLIMBER/*, SWERVEROTATERS,
  // SWERVESPINNERS);
  public final Command extendCommand = new PowerTelescopingCommand(CLIMBER, INTAKE, -1);
  public final Command retractCommand = new PowerTelescopingCommand(CLIMBER, INTAKE, 1);
  // public final Command stayCommand = new PowerTelescopingCommand(CLIMBER, 0);

  // This constructs the robot container class.
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // == BUTTON BINDINGS == //

    // Swervedrive.exe
    SWERVEROTATERS.setDefaultCommand(
        new RunCommand(
            () ->
                SWERVEROTATERS.rotateMotors(
                    shopper.getRawAxis(TRANSLATIONAL_HORIZONTAL_AXIS),
                    shopper.getRawAxis(TRANSLATIONAL_VERTICAL_AXIS),
                    shopper.getRawAxis(ROTATIONAL_HORIZONTAL_AXIS),
                    GYRO.getYaw()),
            SWERVEROTATERS));
    SWERVESPINNERS.setDefaultCommand(
        new RunCommand(
            () ->
                SWERVESPINNERS.spinMotors(
                    shopper.getRawAxis(TRANSLATIONAL_HORIZONTAL_AXIS),
                    shopper.getRawAxis(TRANSLATIONAL_VERTICAL_AXIS),
                    shopper.getRawAxis(ROTATIONAL_HORIZONTAL_AXIS),
                    SWERVEROTATERS.getAngle(
                        shopper.getRawAxis(TRANSLATIONAL_HORIZONTAL_AXIS),
                        shopper.getRawAxis(TRANSLATIONAL_VERTICAL_AXIS),
                        GYRO.getYaw())),
            SWERVESPINNERS));
    GYRO.setDefaultCommand(new RunCommand(() -> GYRO.getState(), GYRO));
    // INTAKE.setDefaultCommand(new RunCommand(() -> INTAKE.intake(), INTAKE));
    // COMPRESSOR.setDefaultCommand( new RunCommand(() -> COMPRESSOR.getSetCompressorStatus(),
    // COMPRESSOR));
    CLIMBER.setDefaultCommand(
        new RunCommand(
            () ->
                CLIMBER.supplyTelescoping(
                    operator.getRawAxis(TRANSLATIONAL_VERTICAL_AXIS),
                    operator.getRawAxis(ROTATIONAL_VERTICAL_AXIS))));

    // Catapult
    lowerCatapultButton.whenHeld(lowerCatapultCommand);
    releaseCatapultButton.whenPressed(releaseCatapultCommand);
    // retractCatapultButton.whenPressed(retractShooterCommand);
    // alignCatapultButton.whenHeld(alignCatapultCommand);

    // Intake
    intakeButton.whileHeld(intakeCommand);
    raiseIntakeButton.whenPressed(raiseIntakeCommand);

    // Gyro
    gyroResetButton.whenPressed(gyroResetCommand);

    // Climber
    // climbButton.whenHeld(climbSequence);

    // extend.whenHeld(extendCommand);
    // retract.whenHeld(retractCommand);

    // stay.whenHeld(stayCommand);
  }

  public void initailizeAutoChooser(SendableChooser<Command> chooser) {
    tenPoint = new Pos1(getRotaters(), getSpinners(), getGyro(), getCatapult(), getIntake());
    sixPoint = new sixpointer(getRotaters(), getSpinners(), getGyro(), getCatapult(), getIntake());
    waitForTeleOp = new DoNothing();
    chooser.addOption("6 Point", sixPoint);
    chooser.addOption("10 point", tenPoint);
    chooser.addOption("Do Nothing", waitForTeleOp);
  }
}
