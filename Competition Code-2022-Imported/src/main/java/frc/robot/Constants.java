/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

  // === PORTS === //

  // Swervedrive Ports
  public static final int MOTOR_PORT_1 = 1;
  public static final int MOTOR_PORT_2 = 3;
  public static final int MOTOR_PORT_3 = 5;
  public static final int MOTOR_PORT_4 = 7;
  public static final int ROTATOR_PORT_1 = 2;
  public static final int ROTATOR_PORT_2 = 4;
  public static final int ROTATOR_PORT_3 = 6;
  public static final int ROTATOR_PORT_4 = 8;

  // Intake Ports
  public static final int INTAKE_MOTOR_PORT = 7;
  public static final int INTAKE_PISTON_PORT_1 = 10; // CHECK
  public static final int INTAKE_PISTON_PORT_2 = 11;
  public static final int INTAKE_PISTON_PORT_3 = 10;
  public static final int INTAKE_PISTON_PORT_4 = 11;

  // Climber Ports
  public static final int TELESCOPE_PORT_1 = 1;
  public static final int TELESCOPE_PORT_2 = 1;
  public static final int SECONDARY_PORT_1 = 1;
  public static final int SECONDARY_PORT_2 = 1;

  // Catapult Ports
  public static final int SHOOTER_MOTOR_PORT_1 = 7;
  public static final int SHOOTER_PISTON_PORT_1 = 5;
  public static final int SHOOTER_PISTON_PORT_2 = 6;
  public static final int SHOOTER_PISTON_PORT_3 = 3; // CCONFIRM LATER UNTESTED
  public static final int SHOOTER_PISTON_PORT_4 = 2;

  // Gyro Port
  public static final int GYRO_PORT = 9;

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

  // Intake
  public static final int INTAKE_BUTTON = RIGHT_BUMPER;
  public static final int RAISE_INTAKE_BUTTON = LEFT_BUMPER;

  // Catapult
  public static final int LOWERCATAPULT_BUTTON = BUTTON_X;
  public static final int RELEASECATAPULT_BUTTON = BUTTON_Y;
  public static final int RETRACTCATAPULTBUTTON = BUTTON_B;
  public static final int ALIGNCATAPULT_BUTTON = BUTTON_A;

  // Climber
  public static final int CLIMB_BUTTON = BUTTON_B;

  // === EXTRAS === //

  // Swervedrive Constants
  public static final double CONTROLLER_SENSITIVITY = 0.02;
  public static final double GEAR_RATIO = 12.8;
  public static final double GEAR_RATIO_SPINNER = 6.86;
  public static final double UNITS_PER_ROTATION = 2048;

  // Speed constants
  public static final double INTAKE_SPEED = -0.5;
  public static final double SHOOTER_PISTON_SPEED = 0.5;
  public static final double CATAPULT_SPEED = 0.9;

  // Time Constants
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
  public static final double ANGLE_ERROR_TOLERANCE = 10;
  public static final double AUTO_ROTATE_SPEED = 0.15;
  public static final double AUTO_ALIGN_SPEED = 0.1;
  public static final double AUTO_ALIGN_ERROR = 5;
}
