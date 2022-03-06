// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Cargo Subsystem enables cargo (rubber balls) to be collected and deposited in the lower hub.
 * Actuators
 *   2 pneumatic cylinders to lower/raise collection arm
 *   1 motor for ball injest
 */
public class CargoSubsystem extends SubsystemBase {
  private final CANSparkMax intakeRoller = new CANSparkMax(Constants.INTAKE_ROLLER_CAN, MotorType.kBrushed);
  private final DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_WRIST_PCM, Constants.BACKWARD_CHANNEL_WRIST_PCM);
  private final DoubleSolenoid elbow = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_ELBOW_PCM, Constants.BACKWARD_CHANNEL_ELBOW_PCM);

  // The number of 20ms ticks required to extend or contract a cylinder
  // 50 X 20ms = 1000ms = 1 second (Need to validate this is the right duration)
  private int ACTUATION_TICKS = 50;

  // These 2 variables count down from ACTUATION_TICKS to zero once we start
  // moving the actuator. Once the counter is <= 0, we know it's done moving.
  private int wristActuationTickCounter = 0;
  private int elbowActuationTickCounter = 0;

  /** Creates a new subsystem. */
  public CargoSubsystem() {
    addChild("Wrist", wrist);
    addChild("Elbow", elbow);
  }

  int errorMessageReporting = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowActuationTickCounter--;
    wristActuationTickCounter--;

    errorMessageReporting++;
    if(errorMessageReporting % 50 == 0) {
      REVLibError error = intakeRoller.getLastError();
      System.out.println("INTAKE ROLLER Last error: " + error.name());
      short stickyFaults = intakeRoller.getStickyFaults();
      System.out.println("INTAKE ROLLER Sticky faults: " + stickyFaults);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Disable the pneumatics so the arms can be moved freely by humans
   */
  public void disable() {
    elbow.set(Value.kOff);
    wrist.set(Value.kOff);
  }

  // ----------------------------------------------------------------------
  //
  //            ELBOW
  //
  //
  
  /**
   * Check the elbow is completely down
   * @return
   */
  public boolean isElbowDown() {
    return elbow.get() == Value.kForward && elbowActuationTickCounter <= 0;
  }

  public boolean isElbowUp() {
    return elbow.get() == Value.kReverse && elbowActuationTickCounter <= 0;
  }

  public void raiseElbow() {
    if(isElbowUp()){
      return;
    }
    elbow.set(Value.kReverse);
    elbowActuationTickCounter = ACTUATION_TICKS;
  }

  public void lowerElbow() {
    if(isElbowDown()){
      return;
    }
    elbow.set(Value.kForward);
    elbowActuationTickCounter = ACTUATION_TICKS;
  }


  // ----------------------------------------------------------------------
  //
  //          WRIST
  //
  //

  public boolean isWristUp() {
    return wrist.get() == Value.kReverse && wristActuationTickCounter <= 0;
  }

  public boolean isWristDown() {
    return wrist.get() == Value.kForward && wristActuationTickCounter <= 0;
  }

  public void raiseWrist() {
    if(isWristUp()){
      return;
    }
    // make sure to only raise the wrist when elbow is down
    if(isElbowDown()) {
      wrist.set(Value.kReverse);
      wristActuationTickCounter = ACTUATION_TICKS;
    } else {
      System.out.println("ERROR: INVALID REQUEST to raise cargo wrist, but the elbow is up");
    }
  }

  public void lowerWrist() {
    if(isWristDown()){
      return;
    }
    wrist.set(Value.kForward);
    wristActuationTickCounter = ACTUATION_TICKS;
  }

  // ----------------------------------------------------------------------
  //
  //          INTAKE ROLLER
  //
  //

  /**
   * Use 1 for intake, -1 for eject, 0 to stop motor
   * @param speed
   */
  public void setIntakeRoller(double speed) {
    // once you let go of the button it turns off
    intakeRoller.set(speed);
  }

}
