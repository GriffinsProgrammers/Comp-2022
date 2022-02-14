// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import static frc.robot.Constants.*;

public class Catapult extends SubsystemBase {

  // Motors
  private WPI_VictorSPX catapult_motor_1;

  // Pistons
  private DoubleSolenoid shooter_1, shooter_2;

  // Constructor
  public Catapult() {
    catapult_motor_1 = new WPI_VictorSPX(SHOOTER_MOTOR_PORT_1);
    shooter_1 =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, SHOOTER_PISTON_PORT_1, SHOOTER_PISTON_PORT_2);
    // shooter_2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_PISTON_PORT_3,
    // SHOOTER_PISTON_PORT_4);
  }

  /**
   * @param speed [-1.0, 1.0]
   */

  // This function sets the speed for the winding catapult motor.
  public void setSpeed() {
    catapult_motor_1.set(ControlMode.PercentOutput, CATAPULT_SPEED);
  }

  // This function stops the winding catapult motor.
  public void motorsOff() {
    catapult_motor_1.set(ControlMode.PercentOutput, 0);
  }

  // This function pushes the catapult piston forward.
  public void extendPiston() {
    shooter_1.set(DoubleSolenoid.Value.kForward);
    // shooter_2.set(DoubleSolenoid.Value.kForward);
  }

  // This function pulls the catapult pistion back.
  public void retractPiston() {
    shooter_1.set(DoubleSolenoid.Value.kReverse);
    // shooter_2.set(DoubleSolenoid.Value.kReverse);
  }
  // This function gets the current state of the piston
  public DoubleSolenoid.Value getPistonState() {
    return shooter_1.get();
  }
  // TODO: Make limit switch (ASK SUFAN)

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
