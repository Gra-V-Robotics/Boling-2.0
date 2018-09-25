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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmSubsystemPID extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonSRX armMotor1, armMotor2, armMotor3;
  Encoder encoder; 
  double tolerance = 0.005; 
  public static double kP = 0.003;
  public static double kI = 0.005;
  public static double kD = 0.005;
  
  public ArmSubsystemPID(){
    super("Arm", kP, kI, kD);
    getPIDController().setPID(kP, kI, kD);
    setAbsoluteTolerance(tolerance);
    armMotor1 = new TalonSRX(RobotMap.armMotor1);
    armMotor2 = new TalonSRX(RobotMap.armMotor2);
    armMotor3 = new TalonSRX(RobotMap.armMotor3);  
    encoder = new Encoder(RobotMap.encoderChannelA, RobotMap.encoderChannelB, false, EncodingType.k4X);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    return encoder.pidGet();
  }

  @Override
  protected void usePIDOutput(double output) {
    armMotor1.set(ControlMode.PercentOutput, output);
    armMotor2.follow(armMotor1);
    armMotor3.follow(armMotor1);
  }
}
