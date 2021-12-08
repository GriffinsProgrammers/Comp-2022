/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class SwerveSpinners extends SubsystemBase {

  /** These are the variables for the SwerveSpinners subsytem. */
  public static final double MM_TO_IN = 0.0393701;
  public static final double WHEEL_TO_WHEEL_DIAMETER_INCHES = 320 * MM_TO_IN;
  public static final double WHEE7L_DIAMETER_INCHES = 4;
  public static final double MOTOR_POWER = 0.5;

  private WPI_TalonFX bRMotor, bLMotor, fRMotor, fLMotor;
  private SpeedControllerGroup bR, bL, fR, fL;
  
  //This is the constructor for this subsytem.
  public SwerveSpinners() {
    bRMotor = new WPI_TalonFX(MOTOR_PORT_4);
    bLMotor = new WPI_TalonFX(MOTOR_PORT_3);
    fRMotor = new WPI_TalonFX(MOTOR_PORT_1);
    fLMotor = new WPI_TalonFX(MOTOR_PORT_2);

    bR = new SpeedControllerGroup(bRMotor);
    bL = new SpeedControllerGroup(bLMotor);
    fR = new SpeedControllerGroup(fRMotor);
    fL = new SpeedControllerGroup(fLMotor);
  }


  //This function is the default command for the swervedrive motor spinners.
  public void spinMotors(double horizontal, double vertical, double rotationHorizontal){
    //This -1 is due to how the vertical axis works on the controller. 
    vertical *= -1;
    double r = Math.sqrt(L*L + W*W);


    double a = (horizontal-rotationHorizontal) * (L/r);
    double b = (horizontal + rotationHorizontal) * (L/r);
    double c = (vertical - rotationHorizontal) * (W/r);
    double d = (vertical+rotationHorizontal) * (W/r);
    //check L & W with CAD people, yes...
    double backRightSpeed = Math.sqrt(a*a + d*d);
    double backLeftSpeed = Math.sqrt(a*a + c*c);
    double frontRightSpeed = Math.sqrt(b*b + d*d);
    double frontLeftSpeed = Math.sqrt(b*b + c*c);

    bR.set(MOTOR_POWER*backRightSpeed);
    bL.set(MOTOR_POWER*backLeftSpeed);
    fR.set(MOTOR_POWER*frontRightSpeed);
    fL.set(MOTOR_POWER*frontLeftSpeed);

    /*
    double trueSpinSpeed = ((Math.sqrt(Math.pow(horizontal, 2)+Math.pow(vertical,2)))/(Math.sqrt(2))*MOTOR_POWER);
    die_emre.set(-trueSpinSpeed*(getSpinDirection(vertical)));
    */
  }

  /**
   * This function is for getting the direction for the spin of a wheel. The current angles are useless right now.
   * However, it may be of a use if my thinking is wrong, so I did not erase it. 
  */
  private double getSpinDirection(double vertical){
    return 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

}