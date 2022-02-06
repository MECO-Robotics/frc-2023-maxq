// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 2022 Robot:
 *    Actuators
 *        2 pneumatic cylinders to lower/raise secondary arm
 *        1 motor for lower arm winch
 *        1 motor for upper arm winch
 *    Sensors
 *        1 Encoder for each winch
 */
public class ClimbingSubsystem extends SubsystemBase {

  // This solenoid controls the forward and reverse movement of the pneumatic
  // cylinder that raises (forward) or lowers (contracts) the upper arm
  // Using the CTRE Pneumatics Control Module (PCM). 
  // Need to move these numbers to Constants.
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, 
    Constants.DOUBLE_SOLENOID_FWD, 
    Constants.DOUBLE_SOLENOID_REV);

  private final TalonSRX lowerLeftArmWinch = new TalonSRX(100);
  private final TalonSRX lowerRightArmWinch = new TalonSRX(100);
  private final TalonSRX upperLeftArmWinch = new TalonSRX(100);
  private final TalonSRX upperRightArmWinch = new TalonSRX(100);

  // The number of ticks required to wind in the lower arm winch
  double lowerArmWinchInEncoderTicks = 1000;

  // The number of ticks required to wind in the lower arm winch
  double upperArmWinchInEncoderTicks = 1000;



  // Need to remove & test. Docs indicate this isn't needed
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  
  /** Creates a new subsystem. */
  public ClimbingSubsystem() {

    // Automatically turn on compressor when pressure switch indicates pressure is too low and off when pressure is too high
    // This line also turns on the compressor when the robot is enabled. It requires a pressure sensor connected to the 
    // Pneumatics Control Module (PCM)
    // This is the default behavior, so we'll be deleting this line once we can test.
    compressor.enableDigital();

    // We're using the VersaPlanetary Integrated Encoder (https://www.vexrobotics.com/217-5046.html) that is directly
    // connected to the Talon SRX Motor Controller (https://store.ctr-electronics.com/talon-srx/)
    lowerLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 250);
    lowerRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 250);
    upperLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 250);
    upperRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 250);


    addChild("Compressor", compressor);
    addChild("Double Solenoid", doubleSolenoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }


  //
  //                

  public void extendPneumaticArm() {
    doubleSolenoid.set(Value.kForward);
  }

  public void contractPneumaticArm() {
    doubleSolenoid.set(Value.kReverse);
  }
  public void disengagePneumaticArm() {
    doubleSolenoid.set(Value.kOff);
  }
  
  public void windLowerArmWinch() {
    // Use the TalonSRX builtin Position control mode, which allows us to simply tell
    // the motor controller, using the attached encoder, how many ticks to move the motor
    lowerLeftArmWinch.set(TalonSRXControlMode.Position, lowerArmWinchInEncoderTicks);
    lowerRightArmWinch.set(TalonSRXControlMode.Position, lowerArmWinchInEncoderTicks);
  }
  public void unwindLowerArmWinch() {
    lowerLeftArmWinch.set(TalonSRXControlMode.Position, -lowerArmWinchInEncoderTicks);
    lowerRightArmWinch.set(TalonSRXControlMode.Position, -lowerArmWinchInEncoderTicks);
  }
  
  public void windUpperArmWinch() { }
  public void unwindUpperArmWinch() { }













  public void control(Value value) {
    doubleSolenoid.set(value);
  }
  
  public void forward() {
    doubleSolenoid.set(Value.kForward);
  }

  public void reverse() {
    doubleSolenoid.set(Value.kReverse);
  }

  public void off() {
    doubleSolenoid.set(Value.kOff);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
