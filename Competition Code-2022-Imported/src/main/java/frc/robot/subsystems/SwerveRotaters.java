// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveRotaters extends SubsystemBase {
  // This is the sub-class for the swervedrive.
  // This class is concerned with the rotation motors for the wheels.

  // == INITIALIZATION == //

  /** These are the variables that are created for this subsytem.. */
  private WPI_TalonFX fRRotater, fLRotater, bLRotater, bRRotater;

  public final double ENCODER_PULSES_PER_ROTATION = 2048;
  public final double ROTATION_POW = 25;

  // This is the constructor where the rotater motors are created and are reset.
  // In addition this is where the PID initilization occurs for each rotater motor
  // The sensor configuration is also set in this constructor.

  public SwerveRotaters() {

    fRRotater = new WPI_TalonFX(ROTATOR_PORT_1);
    fLRotater = new WPI_TalonFX(ROTATOR_PORT_2);
    bLRotater = new WPI_TalonFX(ROTATOR_PORT_3);
    bRRotater = new WPI_TalonFX(ROTATOR_PORT_4);

    // Current Limiting for all motors in order to avoid Brownouts.
    limitMotorCurrent();

    // Sensor config and encoder reset for all motors

    fRRotater.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    fLRotater.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    bLRotater.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    bRRotater.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    resetEncoders();
    configMotors();
  }

  // == FUNCTIONS == //

  // This function resets the encoders of all motors and is called in the constructor.
  public void resetEncoders() {
    fRRotater.setSelectedSensorPosition(0);
    fLRotater.setSelectedSensorPosition(0);
    bLRotater.setSelectedSensorPosition(0);
    bRRotater.setSelectedSensorPosition(0);
  }

  // This function limits the current that each motor is drawing
  public void limitMotorCurrent() {
    fRRotater.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 1));
    fRRotater.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 1));

    fLRotater.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 1));
    fLRotater.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 1));

    bLRotater.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 1));
    bLRotater.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 1));

    bRRotater.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 1));
    bRRotater.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 1));
  }

  // This function configures the motors
  public void configMotors() {
    // Config front right rotater
    fRRotater.configFactoryDefault();
    fRRotater.set(ControlMode.Velocity, 0);
    fRRotater.config_kP(0, kGains.kP);
    fRRotater.config_kI(0, kGains.kI);
    fRRotater.config_kD(0, kGains.kD);
    fRRotater.config_kF(0, kGains.kF);
    fRRotater.setSensorPhase(true);

    // Config front left rotater
    fLRotater.configFactoryDefault();
    fLRotater.set(ControlMode.Velocity, 0);
    fLRotater.config_kP(0, kGains.kP);
    fLRotater.config_kI(0, kGains.kI);
    fLRotater.config_kD(0, kGains.kD);
    fLRotater.config_kF(0, kGains.kF);
    fLRotater.setSensorPhase(true);

    // Config back right rotater
    bLRotater.configFactoryDefault();
    bLRotater.set(ControlMode.Velocity, 0);
    bLRotater.config_kP(0, kGains.kP);
    bLRotater.config_kI(0, kGains.kI);
    bLRotater.config_kD(0, kGains.kD);
    bLRotater.config_kF(0, kGains.kF);
    bLRotater.setSensorPhase(true);

    // Config back left rotater
    bRRotater.configFactoryDefault();
    bRRotater.set(ControlMode.Velocity, 0);
    bRRotater.config_kP(0, kGains.kP);
    bRRotater.config_kI(0, kGains.kI);
    bRRotater.config_kD(0, kGains.kD);
    bRRotater.config_kF(0, kGains.kF);
    bRRotater.setSensorPhase(true);
  }

  // This function converts a provided angle to the encoder pulse value for the motors.
  public double angleToPulse(double angle) {
    return angle * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360;
  }

  // This function converts a provided angle to the angle relative to the front of the robot.
  // Then the function converts the relative angle to the encoder pulse value for the motors.
  public double angleToPulse(double angle, double yaw) {
    return angle(angle, yaw) * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360;
  }

  // This function calculates the angle by using the coordinates defined on the controller.
  // Then the angle is converted into a relative angle (relative to the front of the robot).
  // Then the angle is converted into the encoder pulse value for the motors.
  public double angleToPulse(double horizontal, double vertical, double yaw) {
    return angle(horizontal, vertical, yaw) * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360;
  }

  // This function converts a provided angle to the angle relative to the front of the robot.
  private double angle(double angle, double yaw) {
    // This uses the yaw of the robot in order to calculate the angle we want to turn relative to
    // the front
    // of the robot, which is the 0 point of the encoders.
    if (angle >= yaw) angle -= yaw;
    else angle = 360 - (yaw - angle);
    return (angle % 360);
  }

  // This function calculates the angle by using the coordinates defined on the controller.
  // Then the angle is converted into a relative angle (relative to the front of the robot).
  private double angle(double horizontal, double vertical, double yaw) {
    double angle = 0;
    angle = Math.toDegrees(Math.atan(-horizontal / vertical));
    boolean horizontalIsPositive = (horizontal > 0);
    boolean verticalIsPositive = (vertical > 0);

    if (!horizontalIsPositive && !verticalIsPositive) {
      angle = (90 - (-angle)) + 90;
    } else if (horizontalIsPositive && !verticalIsPositive) {
      angle = (angle + 180);
    } else if (horizontalIsPositive && verticalIsPositive) {
      angle = 360 + angle;
    }
    // This uses the yaw of the robot in order to calculate the angle we want to turn relative to
    // the front
    // of the robot, which is the 0 point of the encoders.
    if (angle >= yaw) angle -= yaw;
    else angle = 360 - (yaw - angle);
    return (angle % 360);
  }

  // This method calls the method before this one with negative of the second value provided.
  // This method is called in Robot Container.
  public double getAngle(double horizontal, double vertical, double yaw) {
    return angle(horizontal, -vertical, yaw);
  }

  // This is the default method used in this class for swervedrive.
  public void rotateMotors(
      double horizontal, double vertical, double rotationHorizontal, double yaw) {
    // This -1 is because the vertical axis provided by the controller is reversed.
    vertical *= -1;
    // First we determine the goal pulse of the angle relative to the front of the robot.
    double goal = angleToPulse(horizontal, vertical, yaw);
    // This is the same thing above, but in angle rather than pulses.
    double angle = angle(horizontal, vertical, yaw);
    // The 2 booleans below are present for the purpose of sensitivity.
    // Also they are useful for distinguishing between our 3 different swerve movements.
    boolean isRotating = Math.abs(rotationHorizontal) >= CONTROLLER_SENSITIVITY;
    boolean isTranslating =
        (Math.sqrt((Math.pow(vertical, 2) + Math.pow(horizontal, 2))) >= CONTROLLER_SENSITIVITY);

    // This if statement is for only translation swerve.
    if (isTranslating && !isRotating) {
      // Using the PID initialized motors, we can use the position control mode.
      // The motors' positions are set to the goal pulses, alining all motors with the wanted
      // direction.
      fRRotater.set(ControlMode.Position, goal);
      fLRotater.set(ControlMode.Position, goal);
      bLRotater.set(ControlMode.Position, goal);
      bRRotater.set(ControlMode.Position, goal);
    }

    // This if statement is for only rotation swerve.
    else if (isRotating && !isTranslating) {
      // The motors are all set to predetermined angles that are best suited for only rotation.
      // In this case, since our swerve is a square, that happens at the following angles.
      fRRotater.set(ControlMode.Position, 45 * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360);
      fLRotater.set(ControlMode.Position, 135 * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360);
      bLRotater.set(ControlMode.Position, 225 * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360);
      bRRotater.set(ControlMode.Position, 315 * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO) / 360);
    }

    // This if statement is for translation and rotation swerve.
    else if (isRotating && isTranslating) {
      // This is split into 4 sections, named zones 1 through 4.
      // The zones are all relative to the front of the robot.
      // If we want to add rotation to an already translating swerve, we use deflection.
      // We deflect the angle of two wheels clockwise(C) and the others counter-clockwise(CC).
      // This deflection angle is mirrored and it is the same for all motors.
      // Which ones are deflected clockwise(C) and counter-clockwise(CC) is different in the zones.

      // In every orientation, 2 are deflected C, while 2 are deflected CC.
      // Therefore, the directional vectors of the motors have a net direction.
      // This net direction is the same as the original only translational direction.
      // This is due to the 2 to 2 ratio of C and CC deflection.

      // The different orientations in the zones ensure that there will always be net rotation.
      // The deflections create this net rotation.
      // The net rotation magnitude differs slightly depending on your original only translation
      // dir.
      // However, it does not differ greatly, therefore rotation + translation is very smooth.

      // It is also worth noting that our of the multiple different swerve algorithms we tested
      // this method is definitely the smoothest (by far).

      // Zone 1
      // This is quadrant starts from 45 degrees to the right of the front of the robot.
      // It ends at 45 degrees to the left of the front of the robot.
      if (((angle >= 0) && (45 > angle)) || ((360 > angle) && (angle >= 315))) {
        fRRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        fLRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        bLRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        bRRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
      }

      // Zone 2
      // This is quadrant starts from 45 degrees to the left of the front of the robot.
      // It ends at 135 degrees to the left of the front of the robot.
      else if ((angle >= 45) && (135 > angle)) {
        bLRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        fLRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        fRRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        bRRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
      }

      // Zone 3
      // This is quadrant starts from 135 degrees to the left of the front of the robot.
      // It ends at 225 degrees to the left of the front of the robot.
      else if ((angle >= 135) && (225 > angle)) {
        bLRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        bRRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        fLRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        fRRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
      }

      // Zone 4
      // This is quadrant starts from 225 degrees to the left of the front of the robot.
      // It ends at 315 degrees to the left of the front of the robot(aka. 45 degrees to the right).
      else if ((angle >= 225) && (315 > angle)) {
        fRRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        bRRotater.set(
            ControlMode.Position,
            ((angle - rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        bLRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
        fLRotater.set(
            ControlMode.Position,
            ((angle + rotationHorizontal * ROTATION_POW) % 360)
                * (ENCODER_PULSES_PER_ROTATION * GEAR_RATIO)
                / 360);
      }
    }
  }

  // This method sets the wheel direction of the 4 motors, with provided encoder pulse values.
  public void setWheelDirection(double fR, double fL, double bR, double bL) {
    fRRotater.set(ControlMode.Position, fR);
    fLRotater.set(ControlMode.Position, fL);
    bRRotater.set(ControlMode.Position, bR);
    bLRotater.set(ControlMode.Position, bL);
  }

  // This method is for autonomous.
  // To check if each motor has reached the error boundary of a provided value.
  public boolean reachedPosition(double a, double b, double c, double d) {
    if (checkError(fRRotater, a)
        && checkError(fLRotater, b)
        && checkError(bRRotater, c)
        && checkError(bLRotater, d)) {
      return true;
    }
    return false;
  }

  // This method is for autonomous.
  // To check if an individual motor has reached the error boundary of the provided value.
  private boolean checkError(WPI_TalonFX motor, double d) {
    return motor.getSelectedSensorPosition() < d + ROTATOR_ERROR_TOLERANCE
        && motor.getSelectedSensorPosition() > d - ROTATOR_ERROR_TOLERANCE;
  }

  // This method stops all motors.
  public void stop() {
    fRRotater.set(ControlMode.PercentOutput, 0);
    fLRotater.set(ControlMode.PercentOutput, 0);
    bRRotater.set(ControlMode.PercentOutput, 0);
    bLRotater.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
