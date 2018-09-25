/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonSRX armMotor1, armMotor2, armMotor3;
  
  public ArmSubsystem(){
    armMotor1 = new TalonSRX(RobotMap.armMotor1);
    armMotor2 = new TalonSRX(RobotMap.armMotor2);
    armMotor3 = new TalonSRX(RobotMap.armMotor3);  
  }

  public void armControl(double power){
    armMotor1.set(ControlMode.PercenOutput, power);
    armMotor2.set(ControlMode.Follower, armMotor2);
    armMotor3.set(ControlMode.Follower, armMotor3);      
  };
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }  
}
