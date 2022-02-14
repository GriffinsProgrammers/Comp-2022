/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// import org.graalvm.compiler.asm.sparc.SPARCAssembler.Br;

// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveSpinners extends SubsystemBase {

  /** These are the variables for the SwerveSpinners subsytem. */
  public static final double WHEEL_DIAMETER_INCHES = 4;

  public static final double SPEED_MULTIPLIER = 0.35;
  public static final double ROTATION_COEFFICIENT = 0.35;
  private WPI_TalonFX bRMotor, bLMotor, fRMotor, fLMotor;
  // private SpeedControllerGroup bR, bL, fR, fL;
  private MotorControllerGroup bR, bL, fR, fL;

  // This is the constructor for this subsytem.

  public SwerveSpinners() {

    fRMotor = new WPI_TalonFX(MOTOR_PORT_1);
    fLMotor = new WPI_TalonFX(MOTOR_PORT_2);
    bLMotor = new WPI_TalonFX(MOTOR_PORT_3);
    bRMotor = new WPI_TalonFX(MOTOR_PORT_4);

    fR = new MotorControllerGroup(fRMotor);
    fL = new MotorControllerGroup(fLMotor);
    bR = new MotorControllerGroup(bRMotor);
    bL = new MotorControllerGroup(bLMotor);

    limitMotorCurrent();
    configPID();
  }

  public void limitMotorCurrent() {
    fLMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 0.1));
    fLMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 0.1));

    fRMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 0.1));
    fRMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 0.1));

    bRMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 0.1));
    bRMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 0.1));

    bLMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 21, 0.1));
    bLMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 0.1));
  }

  // This is the initialization method for PID config.
  public void configPID() {
    resetEncoders();

    fRMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    fLMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    bLMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    bRMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    fLMotor.configFactoryDefault();
    fLMotor.set(ControlMode.Velocity, 0);
    fLMotor.config_kP(0, jGains.kP);
    fLMotor.config_kI(0, jGains.kI);
    fLMotor.config_kD(0, jGains.kD);
    fLMotor.config_kF(0, jGains.kF);
    fLMotor.setSensorPhase(true);

    fRMotor.configFactoryDefault();
    fRMotor.set(ControlMode.Velocity, 0);
    fRMotor.config_kP(0, jGains.kP);
    fRMotor.config_kI(0, jGains.kI);
    fRMotor.config_kD(0, jGains.kD);
    fRMotor.config_kF(0, jGains.kF);
    fRMotor.setSensorPhase(true);

    bLMotor.configFactoryDefault();
    bLMotor.set(ControlMode.Velocity, 0);
    bLMotor.config_kP(0, jGains.kP);
    bLMotor.config_kI(0, jGains.kI);
    bLMotor.config_kD(0, jGains.kD);
    bLMotor.config_kF(0, jGains.kF);
    bLMotor.setSensorPhase(true);

    bRMotor.configFactoryDefault();
    bRMotor.set(ControlMode.Velocity, 0);
    bRMotor.config_kP(0, jGains.kP);
    bRMotor.config_kI(0, jGains.kI);
    bRMotor.config_kD(0, jGains.kD);
    bRMotor.config_kF(0, jGains.kF);
    bRMotor.setSensorPhase(true);
  }

  // This method resets the encoder values.
  public void resetEncoders() {
    fRMotor.setSelectedSensorPosition(0);
    fLMotor.setSelectedSensorPosition(0);
    bLMotor.setSelectedSensorPosition(0);
    bRMotor.setSelectedSensorPosition(0);
  }

  // This function is the default command for the swervedrive spinners.
  public void spinMotors(
      double horizontal, double vertical, double rotationHorizontal, double angle) {
    // This -1 is due to how the vertical axis works on the controller.
    vertical *= -1;

    double r =
        (Math.pow(Math.sqrt(horizontal * horizontal + vertical * vertical), 1) * SPEED_MULTIPLIER);
    double backRightSpeed = 0;
    double backLeftSpeed = 0;
    double frontRightSpeed = 0;
    double frontLeftSpeed = 0;

    boolean isRotating = Math.abs(rotationHorizontal) >= CONTROLLER_SENSITIVITY;
    boolean isTranslating =
        (Math.sqrt((Math.pow(vertical, 2) + Math.pow(horizontal, 2))) >= CONTROLLER_SENSITIVITY);

    if (!isRotating && isTranslating) {
      frontRightSpeed = r;
      backLeftSpeed = r;
      backRightSpeed = r;
      frontLeftSpeed = r;

    } else if (isRotating && !isTranslating) {
      backRightSpeed = -rotationHorizontal * ROTATION_COEFFICIENT;
      frontRightSpeed = -rotationHorizontal * ROTATION_COEFFICIENT;
      backLeftSpeed = -rotationHorizontal * ROTATION_COEFFICIENT;
      frontLeftSpeed = -rotationHorizontal * ROTATION_COEFFICIENT;

    } else if (isRotating && isTranslating) {
      frontRightSpeed = r;
      backLeftSpeed = r;
      backRightSpeed = r;
      frontLeftSpeed = r;
    }

    fR.set(frontRightSpeed);
    fL.set(frontLeftSpeed);
    bR.set(backRightSpeed);
    bL.set(backLeftSpeed);
  }

  // == Auto Commands == //

  public double cmToPulses(double cm) {
    return GEAR_RATIO_SPINNER * UNITS_PER_ROTATION * cm / (WHEEL_DIAMETER_INCHES * 2.54 * Math.PI);
  }

  public void driveDistance(double pulseGoal) {
    bRMotor.set(ControlMode.Position, pulseGoal);
    bLMotor.set(ControlMode.Position, pulseGoal);
    fRMotor.set(ControlMode.Position, pulseGoal);
    fLMotor.set(ControlMode.Position, pulseGoal);
  }

  public boolean reachedPosition(double pulsesDistance) {
    if (checkError(bRMotor, pulsesDistance)
        && checkError(bLMotor, pulsesDistance)
        && checkError(fRMotor, pulsesDistance)
        && checkError(fLMotor, pulsesDistance)) {
      return true;
    }
    return false;
  }

  private boolean checkError(WPI_TalonFX motor, double pulsesDistance) {
    return ((motor.getSelectedSensorPosition() < (pulsesDistance + SPINNER_ERROR_TOLERANCE))
        && (motor.getSelectedSensorPosition() > (pulsesDistance - SPINNER_ERROR_TOLERANCE)));
  }

  public void stop() {
    fRMotor.set(ControlMode.PercentOutput, 0);
    fLMotor.set(ControlMode.PercentOutput, 0);
    bRMotor.set(ControlMode.PercentOutput, 0);
    bLMotor.set(ControlMode.PercentOutput, 0);
  }

  public void runSpinners(double speed) {
    fRMotor.set(ControlMode.PercentOutput, speed);
    fLMotor.set(ControlMode.PercentOutput, speed);
    bRMotor.set(ControlMode.PercentOutput, speed);
    bLMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
