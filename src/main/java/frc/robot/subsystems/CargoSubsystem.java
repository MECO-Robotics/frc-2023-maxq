// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The Cargo Subsystem enables cargo (rubber balls) to be collected and deposited in the lower hub.
 * Actuators
 *   2 pneumatic cylinders to lower/raise collection arm
 *   1 motor for ball injest
 */
public class CargoSubsystem extends SubsystemBase {

  private final Spark intakeRoller = new Spark(Constants.INTAKE_ROLLER_PWM);
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
    addChild("Intake", intakeRoller);
    addChild("Wrist", wrist);
    addChild("Elbow", elbow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowActuationTickCounter--;
    wristActuationTickCounter--;
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
    // make sure that the wrist is down before raising the elbow
    if (isWristDown()) {
      elbow.set(Value.kReverse);
      elbowActuationTickCounter = ACTUATION_TICKS;
    } else {
      System.out.println("ERROR: INVALID REQUEST to raise the elbow, but the wrist is up");
    }
  }

  public void lowerElbow() {
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
    // make sure to only raise the wrist when elbow is down
    if(isElbowDown()) {
      wrist.set(Value.kReverse);
      wristActuationTickCounter = ACTUATION_TICKS;
    } else {
      System.out.println("ERROR: INVALID REQUEST to raise cargo wrist, but the elbow is up");
    }
  }

  public void lowerWrist() {
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
