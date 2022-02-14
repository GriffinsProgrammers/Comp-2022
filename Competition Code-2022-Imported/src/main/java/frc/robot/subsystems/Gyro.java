/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

/** This is the subsytem for our gyroscope */
// Pigeon IMU gyro info
// https://www.ctr-electronics.com/downloads/pdf/Pigeon%20IMU%20User's%20Guide.pdf

public class Gyro extends SubsystemBase {

  // Initialize fields
  private PigeonIMU gyro;
  private double[] ypr = new double[3];

  public enum PigeonState {
    NoComm,
    Initializing,
    Ready,
    UserCalibration,
  };

  // Constructor
  public Gyro() {
    gyro = new PigeonIMU(GYRO_PORT);
    resetValues();
    calibrateGyro();
    getState();
  }

  // This function returns the status of the gyro.
  public ErrorCode getStatus(PigeonIMU.GeneralStatus genStatus) {
    return gyro.getGeneralStatus(genStatus);
  }

  // This function return the yaw of the gyro.
  public double getYaw() {
    gyro.getYawPitchRoll(ypr);
    return (ypr[0] % 360);
  }

  // This function calibrates the gyro.
  public void calibrateGyro() {
    gyro.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
  }

  // This function resets the values of the gyro.
  public void resetValues() {
    gyro.configFactoryDefault();
  }

  // This function is for printing the current state of the gyro.
  // This is important for knowing when the gyro is ready, so swerve can work properly.
  public void getState() {
    if (gyro.getState() == PigeonIMU.PigeonState.Ready) {
      System.out.println("Ready");
    } else if (gyro.getState() == PigeonIMU.PigeonState.UserCalibration) {
      System.out.println("Calibrating");
    } else if (gyro.getState() == PigeonIMU.PigeonState.Initializing) {
      System.out.println("Initializing");
    } else if (gyro.getState() == PigeonIMU.PigeonState.NoComm) {
      System.out.println("No communication with pigeon gyro");
    }
  }

  public double getGyroState() {
    return gyro.getState() == PigeonIMU.PigeonState.Ready ? 1 : 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
