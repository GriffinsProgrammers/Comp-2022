/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS =
      0.617; // FIXME Measure and set trackwidth
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.617; // FIXME Measure and set wheelbase

  public static final int DRIVETRAIN_PIGEON_ID = 9; // FIXME Set Pigeon ID

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR =
      3; // FIXME Set front left module drive motor ID
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR =
      4; // FIXME Set front left module steer motor ID
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER =
      4; // FIXME Set front left steer encoder ID

  public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
      -Math.toRadians(0.0); // FIXME Measure and set front
  // left steer offset

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR =
      1; // FIXME Set front right drive motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR =
      2; // FIXME Set front right steer motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER =
      2; // FIXME Set front right steer encoder ID

  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
      -Math.toRadians(0.0); // FIXME Measure and set front
  // right steer offset

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
  public static final int BACK_LEFT_MODULE_STEER_ENCODER =
      6; // FIXME Set back left steer encoder ID

  public static final double BACK_LEFT_MODULE_STEER_OFFSET =
      -Math.toRadians(0.0); // FIXME Measure and set back left
  // steer offset

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER =
      8; // FIXME Set back right steer encoder ID
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
      -Math.toRadians(0.0); // FIXME Measure and set back
  // right steer offset

  // Intake Ports
  public static final int INTAKE_MOTOR_PORT = 10;
  public static final int INTAKE_PISTON_PORT_1 = 4;
  public static final int INTAKE_PISTON_PORT_2 = 5;

  // Climber Ports
  public static final int TELESCOPE_PORT_1 = 12;
  public static final int TELESCOPE_PORT_2 = 13;
  public static final int SECONDARY_PORT_1 = 1;
  public static final int SECONDARY_PORT_2 = 1;

  // Catapult Ports
  public static final int SHOOTER_MOTOR_PORT_1 = 11;
  public static final int SHOOTER_PISTON_PORT_1 = 7;
  public static final int SHOOTER_PISTON_PORT_2 = 6;

  // Gyro Port

  // === CONTROLLER === //

  // Controller Constants
  public static final int DRIVER_CONTROLLER = 0; // change back to 0
  public static final int OPERATOR_CONTROLLER = 1; // change back to 1

  // == BUTTON CONSTANTS == //

  // Controller Buttons
  public static final int BUTTON_X = 3; // 1 on 001
  public static final int BUTTON_A = 1; // 2 on 001
  public static final int BUTTON_B = 2; // 3 on 001
  public static final int BUTTON_Y = 4; // 4 on 001
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int LEFT_TRIGGER = 7;
  public static final int RIGHT_TRIGGER = 8;
  public static final int BACK_BUTTON = 9;
  public static final int START_BUTTON = 10;

  // Swerve
  public static final int TRANSLATIONAL_HORIZONTAL_AXIS = 0;
  public static final int TRANSLATIONAL_VERTICAL_AXIS = 1;
  public static final int ROTATIONAL_HORIZONTAL_AXIS = 4;
  public static final int ROTATIONAL_VERTICAL_AXIS = 5;

  // Intake
  public static final int INTAKE_BUTTON = RIGHT_BUMPER;
  public static final int RAISE_INTAKE_BUTTON = LEFT_BUMPER;

  // Catapult
  public static final int LOWERCATAPULT_BUTTON = BUTTON_X;

  public static final int RELEASECATAPULT_BUTTON = BUTTON_B;
  // public static final int ALIGNCATAPULT_BUTTON = BUTTON_A;

  // Climber
  public static final int OPERATOR_L3 = 1;
  public static final int OPERATOR_R3 = 4;

  // === EXTRAS === //

  // Swervedrive Constants
  public static final double CONTROLLER_SENSITIVITY = 0.1;
  public static final double GEAR_RATIO = 12.8;
  public static final double GEAR_RATIO_SPINNER = 6.86;
  public static final double UNITS_PER_ROTATION = 2048;

  // Speed constants

  public static final double INTAKE_SPEED = -1;
  public static final double SHOOTER_PISTON_SPEED = 0.5;
  public static final double CATAPULT_SPEED = 0.9;
  public static final double WIND_CATAPULT_TIME = 4;
  public static final double RELEASE_CATAPULT_TIME = 2;

  // Climber Sequence Constants
  public static final double MOVE_SPEED = 0.2;
  public static final double TELESCOPING_INITIALIZE = 0.05;
  public static final double LOWESTPOWER = 0.1;
  public static final double LOWPOWER = 0.25;
  public static final double MIDPOWER = 0.4;
  public static final double HIGHPOWER = 0.6;

  // Swervedrive PID Stuff
  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.0, 0, 0.0);
  public static final Gains jGains = new Gains(0.1, 0.0, 0.0, 0.0, 0, 0.0); // Calibrate

  // Auto Constants
  public static final double ROTATOR_ERROR_TOLERANCE = 100;
  public static final double SPINNER_ERROR_TOLERANCE = 10;
  public static final double ANGLE_ERROR_TOLERANCE = 2.5;
  public static final double AUTO_ROTATE_SPEED = 0.15;
  public static final double AUTO_ALIGN_SPEED = 0.1;
  public static final double AUTO_ALIGN_ERROR = 5;
}
