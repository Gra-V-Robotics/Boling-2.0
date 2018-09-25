/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX rightFrontMotor, rightBackMotor, leftFrontMotor, leftBackMotor;  
  public DriveTrainSubsystem(){
    rightFrontMotor = new TalonSRX(RobotMap.rightFrontMotor);
    rightBackMotor = new TalonSRX(RobotMap.rightBackMotor);
    leftBackMotor = new TalonSRX(RobotMap.leftBackMotor);
    leftFrontMotor = new TalonSRX(RobotMap.leftFrontMotor);    
  }

  public void drivePercentOutput(double leftSpeed, double rightSpeed){
    rightFrontMotor.set(ControlMode.PercentOutput, rightSpeed);
    rightBackMotor.set(ControlMode.PercentOutput, rightSpeed);
    leftFrontMotor.set(ControlMode.PercentOutput, leftSpeed);
    leftBackMotor.set(ControlMode.PercentOutput, leftSpeed);
  }

  public void driveVelocity(double leftVelocity, double rightVelocity){
    rightFrontMotor.set(ControlMode.Velocity, rightVelocity);
    rightBackMotor.set(ControlMode.Velocity, rightVelocity);
    leftFrontMotor.set(ControlMode.Velocity, leftVelocity);
    leftBackMotor.set(ControlMode.Velocity, leftVelocity);
  }

 


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
