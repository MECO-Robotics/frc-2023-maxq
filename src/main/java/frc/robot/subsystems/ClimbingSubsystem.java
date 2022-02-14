// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  private final DoubleSolenoid upperArmSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, 
    Constants.DOUBLE_SOLENOID_FWD, 
    Constants.DOUBLE_SOLENOID_REV);

  private final TalonSRX lowerLeftArmWinch = new TalonSRX(0);
  private final TalonSRX lowerRightArmWinch = new TalonSRX(1);
  private final TalonSRX upperLeftArmWinch = new TalonSRX(2);
  private final TalonSRX upperRightArmWinch = new TalonSRX(3);

  // The number of ticks required to wind in the lower arm winch
  double lowerArmWinchInEncoderTicks = 1000;

  // The number of ticks required to wind in the lower arm winch
  double upperArmWinchInEncoderTicks = 1000;

  
  private MotorStallMonitor lowerRightArmWinchStallMonitor = new MotorStallMonitor(lowerRightArmWinch);
  private MotorStallMonitor lowerLeftArmWinchStallMonitor = new MotorStallMonitor(lowerLeftArmWinch);
  private MotorStallMonitor upperLeftArmWinchStallMonitor = new MotorStallMonitor(upperLeftArmWinch);
  private MotorStallMonitor upperRightArmWinchStallMonitor = new MotorStallMonitor(upperRightArmWinch);


  // Need to remove & test. Docs indicate this isn't needed
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  
  /** Creates a new subsystem. */
  public ClimbingSubsystem() {

    // Automatically turn on compressor when pressure switch indicates pressure is too low and off when pressure is too high
    // This line also turns on the compressor when the robot is enabled. It requires a pressure sensor connected to the 
    // Pneumatics Control Module (PCM)
    // This is the default behavior, so we'll be deleting this line once we can test.
    compressor.enableDigital();

    // Parameters
    //   feedbackDevice
    //      We're using the VersaPlanetary Integrated Encoder (https://www.vexrobotics.com/217-5046.html) that is directly
    //      connected to the Talon SRX Motor Controller (https://store.ctr-electronics.com/talon-srx/).
    //      The Mag Encoder can do either relative (quadrature) or absolute (PWM). The relative approach can sample at a
    //      higher rate, so using that.
    //  pidIdx
    //      0 for Primary closed-loop. 1 for auxiliary closed-loop.
    //      *** Selected 0 ***
    //  timeoutMs
    //      Time to wait for the configuration to complete, and report if an error occurred.
    lowerLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    lowerRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    upperLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    upperRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);

    // We want the winches to run in brake mode so that when we're not winding or unwinding it stays put.
    lowerLeftArmWinch.setNeutralMode(NeutralMode.Brake);
    lowerRightArmWinch.setNeutralMode(NeutralMode.Brake);
    upperLeftArmWinch.setNeutralMode(NeutralMode.Brake);
    upperRightArmWinch.setNeutralMode(NeutralMode.Brake);

    addChild("Compressor", compressor);
    addChild("Upper Arm Solenoid", upperArmSolenoid);
  }

  @Override
  public void periodic() {

    // Check if either lower motor has stalled. If it has, stop the motors and set the winch state
    // accordingly.
    lowerLeftArmWinchStallMonitor.periodic();
    lowerRightArmWinchStallMonitor.periodic();
    if(lowerLeftArmWinchStallMonitor.isStalled() || lowerRightArmWinchStallMonitor.isStalled()) {
      lowerLeftArmWinch.set(ControlMode.PercentOutput, 0);
      lowerRightArmWinch.set(ControlMode.PercentOutput, 0);
      if(lowerArmWinchState == WinchState.Unwinding) {
        lowerArmWinchState = WinchState.Unwound;
      } else if(lowerArmWinchState == WinchState.Winding) {
        lowerArmWinchState = WinchState.Wound;
      }
    }

    // Check if either lower motor has stalled. If it has, stop the motors and set the winch state
    // accordingly.
    upperLeftArmWinchStallMonitor.periodic();
    upperRightArmWinchStallMonitor.periodic();
    if(upperLeftArmWinchStallMonitor.isStalled() || upperRightArmWinchStallMonitor.isStalled()) {
      upperLeftArmWinch.set(ControlMode.PercentOutput, 0);
      upperRightArmWinch.set(ControlMode.PercentOutput, 0);
      if(upperArmWinchState == WinchState.Unwinding){
        upperArmWinchState = WinchState.Unwound;
      } else if(upperArmWinchState == WinchState.Winding) {
        upperArmWinchState = WinchState.Wound;
      }
    }
  }


  //
  //                

  public void extendPneumaticArm() {
    upperArmSolenoid.set(Value.kForward);
  }

  public void contractPneumaticArm() {
    upperArmSolenoid.set(Value.kReverse);
  }
  public void disengagePneumaticArm() {
    upperArmSolenoid.set(Value.kOff);
  }
  

  // ------------------------------------------------------------------------------------

  public enum WinchState { Wound, Unwinding, Unwound, Winding }

  WinchState lowerArmWinchState = WinchState.Wound;
  WinchState upperArmWinchState = WinchState.Wound;

  public WinchState getUpperArmWinchState() {
    if(upperArmWinchState == WinchState.Unwinding && upperLeftArmWinch.getMotorOutputVoltage() == 0 && upperRightArmWinch.getMotorOutputVoltage() == 0) {
      upperArmWinchState = WinchState.Unwound;
    } else if(upperArmWinchState == WinchState.Winding && upperLeftArmWinch.getMotorOutputVoltage() == 0 && upperRightArmWinch.getMotorOutputVoltage() == 0) {
      upperArmWinchState = WinchState.Wound;
    }
  return upperArmWinchState;
}

public WinchState getLowerArmWinchState() {
  
  // Check for stall!!! by monitoring the current draw
  lowerLeftArmWinch.getStatorCurrent();
  
  if(lowerArmWinchState == WinchState.Unwinding && lowerLeftArmWinch.getMotorOutputVoltage() == 0 && lowerRightArmWinch.getMotorOutputVoltage() == 0) {
    lowerArmWinchState = WinchState.Unwound;
  } else if(lowerArmWinchState == WinchState.Winding && lowerLeftArmWinch.getMotorOutputVoltage() == 0 && lowerRightArmWinch.getMotorOutputVoltage() == 0) {
    lowerArmWinchState = WinchState.Wound;
  }
  return lowerArmWinchState;
}

public void windLowerArmWinch() {
    lowerArmWinchState = WinchState.Winding;
    // Use the TalonSRX builtin Position control mode, which allows us to simply tell
    // the motor controller, using the attached encoder, how many ticks to move the motor
    lowerLeftArmWinch.set(TalonSRXControlMode.Position, lowerArmWinchInEncoderTicks);
    lowerRightArmWinch.set(TalonSRXControlMode.Position, lowerArmWinchInEncoderTicks);
  }
  public void unwindLowerArmWinch() {
    lowerArmWinchState = WinchState.Unwinding;
    lowerLeftArmWinch.set(TalonSRXControlMode.Position, -lowerArmWinchInEncoderTicks);
    lowerRightArmWinch.set(TalonSRXControlMode.Position, -lowerArmWinchInEncoderTicks);
  }
  
  public void windUpperArmWinch() { 
    upperArmWinchState = WinchState.Winding;
    upperLeftArmWinch.set(TalonSRXControlMode.Position, upperArmWinchInEncoderTicks);
    upperRightArmWinch.set(TalonSRXControlMode.Position, upperArmWinchInEncoderTicks);
  }
  public void unwindUpperArmWinch() { 
    upperArmWinchState = WinchState.Unwinding;
    upperLeftArmWinch.set(TalonSRXControlMode.Position, -upperArmWinchInEncoderTicks);
    upperRightArmWinch.set(TalonSRXControlMode.Position, -upperArmWinchInEncoderTicks);
  }

  // ------------------------------------------------------------------------------------

  public void control(Value value) {
    upperArmSolenoid.set(value);
  }
  
  public void forward() {
    upperArmSolenoid.set(Value.kForward);
  }

  public void reverse() {
    upperArmSolenoid.set(Value.kReverse);
  }

  public void off() {
    upperArmSolenoid.set(Value.kOff);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
