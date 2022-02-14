// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  /** These are the variables that are created for this subsytem.. */
  public WPI_TalonSRX telescope1, telescope2;

  // This is the constructor for the Climber.
  public Climber() {
    telescope1 = new WPI_TalonSRX(TELESCOPE_PORT_1);
    telescope2 = new WPI_TalonSRX(TELESCOPE_PORT_2);
  }

  // This method supplies a certain output to the telescoping motors.
  public void supplyTelescoping(double power) {
    telescope1.set(ControlMode.PercentOutput, power);
    telescope2.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
