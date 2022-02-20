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
 *        2 pneumatic cylinders to telescoping/raise secondary arm
 *        1 motor for telescoping arm winch
 *        1 motor for rotating arm winch
 *    Sensors
 *        1 Encoder for each winch
 */
public class ClimbingSubsystem extends SubsystemBase {

  // This solenoid controls the forward and reverse movement of the pneumatic
  // cylinder that raises (forward) or telescopings (contracts) the rotating arm
  // Using the CTRE Pneumatics Control Module (PCM). 
  // Need to move these numbers to Constants.
  private final DoubleSolenoid rotatingArmSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, 
    Constants.DOUBLE_SOLENOID_FWD_PCM, 
    Constants.DOUBLE_SOLENOID_REV_PCM);

  private final TalonSRX telescopingLeftArmWinch = new TalonSRX(0);
  private final TalonSRX telescopingRightArmWinch = new TalonSRX(1);
  private final TalonSRX rotatingLeftArmWinch = new TalonSRX(2);
  private final TalonSRX rotatingRightArmWinch = new TalonSRX(3);

  // The number of ticks required to wind in the telescoping arm winch
  double telescopingArmWinchInEncoderTicks = 1000;

  // The number of ticks required to wind in the telescoping arm winch
  double rotatingArmWinchInEncoderTicks = 1000;

  
  private MotorStallMonitor telescopingRightArmWinchStallMonitor = new MotorStallMonitor(telescopingRightArmWinch);
  private MotorStallMonitor telescopingLeftArmWinchStallMonitor = new MotorStallMonitor(telescopingLeftArmWinch);
  private MotorStallMonitor rotatingLeftArmWinchStallMonitor = new MotorStallMonitor(rotatingLeftArmWinch);
  private MotorStallMonitor rotatingRightArmWinchStallMonitor = new MotorStallMonitor(rotatingRightArmWinch);


  // Need to remove & test. Docs indicate this isn't needed
  
  /** Creates a new subsystem. */
  public ClimbingSubsystem() {

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
    telescopingLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    telescopingRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    rotatingLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    rotatingRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);

    // We want the winches to run in brake mode so that when we're not winding or unwinding it stays put.
    telescopingLeftArmWinch.setNeutralMode(NeutralMode.Brake);
    telescopingRightArmWinch.setNeutralMode(NeutralMode.Brake);
    rotatingLeftArmWinch.setNeutralMode(NeutralMode.Brake);
    rotatingRightArmWinch.setNeutralMode(NeutralMode.Brake);

    addChild("rotating Arm Solenoid", rotatingArmSolenoid);
  }

  @Override
  public void periodic() {

    // Check if either telescoping motor has stalled. If it has, stop the motors and set the winch state
    // accordingly.
    telescopingLeftArmWinchStallMonitor.periodic();
    telescopingRightArmWinchStallMonitor.periodic();
    if(telescopingLeftArmWinchStallMonitor.isStalled() || telescopingRightArmWinchStallMonitor.isStalled()) {
      telescopingLeftArmWinch.set(ControlMode.PercentOutput, 0);
      telescopingRightArmWinch.set(ControlMode.PercentOutput, 0);
      if(telescopingArmWinchState == WinchState.Unwinding) {
        telescopingArmWinchState = WinchState.Unwound;
      } else if(telescopingArmWinchState == WinchState.Winding) {
        telescopingArmWinchState = WinchState.Wound;
      }
    }

    // Check if either telescoping motor has stalled. If it has, stop the motors and set the winch state
    // accordingly.
    rotatingLeftArmWinchStallMonitor.periodic();
    rotatingRightArmWinchStallMonitor.periodic();
    if(rotatingLeftArmWinchStallMonitor.isStalled() || rotatingRightArmWinchStallMonitor.isStalled()) {
      rotatingLeftArmWinch.set(ControlMode.PercentOutput, 0);
      rotatingRightArmWinch.set(ControlMode.PercentOutput, 0);
      if(rotatingArmWinchState == WinchState.Unwinding){
        rotatingArmWinchState = WinchState.Unwound;
      } else if(rotatingArmWinchState == WinchState.Winding) {
        rotatingArmWinchState = WinchState.Wound;
      }
    }
  }


  //
  //                

  /**
   * Set the pneumatic cylinder to extend (forward), and let out the 
   * rotating arm winch.
   */
  public void raiseRotatingArm() {
    rotatingArmSolenoid.set(Value.kForward);

    // TOOD: Let out rotating arm winch.
    //    2 options:
    //      A) Switch to Coast mode
    //      B) Run motor at a speed that isn't faster than the cylinder can move the arm
  }

  public void contractPneumaticArm() {
    rotatingArmSolenoid.set(Value.kReverse);
  }
  public void disengagePneumaticArm() {
    rotatingArmSolenoid.set(Value.kOff);
  }
  

  // ------------------------------------------------------------------------------------

  public enum WinchState { Wound, Unwinding, Unwound, Winding }

  WinchState telescopingArmWinchState = WinchState.Wound;
  WinchState rotatingArmWinchState = WinchState.Wound;

  public WinchState getrotatingArmWinchState() {
    if(rotatingArmWinchState == WinchState.Unwinding && rotatingLeftArmWinch.getMotorOutputVoltage() == 0 && rotatingRightArmWinch.getMotorOutputVoltage() == 0) {
      rotatingArmWinchState = WinchState.Unwound;
    } else if(rotatingArmWinchState == WinchState.Winding && rotatingLeftArmWinch.getMotorOutputVoltage() == 0 && rotatingRightArmWinch.getMotorOutputVoltage() == 0) {
      rotatingArmWinchState = WinchState.Wound;
    }
  return rotatingArmWinchState;
}

public WinchState gettelescopingArmWinchState() {
  
  // Check for stall!!! by monitoring the current draw
  telescopingLeftArmWinch.getStatorCurrent();
  
  if(telescopingArmWinchState == WinchState.Unwinding && telescopingLeftArmWinch.getMotorOutputVoltage() == 0 && telescopingRightArmWinch.getMotorOutputVoltage() == 0) {
    telescopingArmWinchState = WinchState.Unwound;
  } else if(telescopingArmWinchState == WinchState.Winding && telescopingLeftArmWinch.getMotorOutputVoltage() == 0 && telescopingRightArmWinch.getMotorOutputVoltage() == 0) {
    telescopingArmWinchState = WinchState.Wound;
  }
  return telescopingArmWinchState;
}

public void windtelescopingArmWinch() {
    telescopingArmWinchState = WinchState.Winding;
    // Use the TalonSRX builtin Position control mode, which allows us to simply tell
    // the motor controller, using the attached encoder, how many ticks to move the motor
    telescopingLeftArmWinch.set(TalonSRXControlMode.Position, telescopingArmWinchInEncoderTicks);
    telescopingRightArmWinch.set(TalonSRXControlMode.Position, telescopingArmWinchInEncoderTicks);
  }
  public void unwindtelescopingArmWinch() {
    telescopingArmWinchState = WinchState.Unwinding;
    telescopingLeftArmWinch.set(TalonSRXControlMode.Position, -telescopingArmWinchInEncoderTicks);
    telescopingRightArmWinch.set(TalonSRXControlMode.Position, -telescopingArmWinchInEncoderTicks);
  }
  
  public void windrotatingArmWinch() { 
    rotatingArmWinchState = WinchState.Winding;
    rotatingLeftArmWinch.set(TalonSRXControlMode.Position, rotatingArmWinchInEncoderTicks);
    rotatingRightArmWinch.set(TalonSRXControlMode.Position, rotatingArmWinchInEncoderTicks);
  }
  public void unwindrotatingArmWinch() { 
    rotatingArmWinchState = WinchState.Unwinding;
    rotatingLeftArmWinch.set(TalonSRXControlMode.Position, -rotatingArmWinchInEncoderTicks);
    rotatingRightArmWinch.set(TalonSRXControlMode.Position, -rotatingArmWinchInEncoderTicks);
  }

  // ------------------------------------------------------------------------------------

  public void control(Value value) {
    rotatingArmSolenoid.set(value);
  }
  
  public void forward() {
    rotatingArmSolenoid.set(Value.kForward);
  }

  public void reverse() {
    rotatingArmSolenoid.set(Value.kReverse);
  }

  public void off() {
    rotatingArmSolenoid.set(Value.kOff);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
