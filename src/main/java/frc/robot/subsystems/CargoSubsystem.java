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
 * 2022 Robot:
 * Actuators
 * 2 pneumatic cylinders to lower/raise collection arm
 * 1 motor for ball injest
 */
public class CargoSubsystem extends SubsystemBase {

  Spark intakeRoller = new Spark(Constants.INTAKE_ROLLER_PWM_CHANNEL);
  DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_WRIST, Constants.BACKWARD_CHANNEL_WRIST);
  DoubleSolenoid elbow = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_ELBOW, Constants.BACKWARD_CHANNEL_ELBOW);

  /** Creates a new subsystem. */
  public CargoSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean isElbowDown() {
    return elbow.get() == Value.kForward;
  }

  public boolean isElbowUp() {
    return elbow.get() == Value.kReverse;
  }

  public void raiseElbow() {
    // make sure that the wrist is down before raising the elbow
    if (isWristDown()) {
      elbow.set(Value.kReverse);
    } else {
      System.out.println("ERROR: INVALID REQUEST to raie the elbow, but the wrist is up");
    }
  }

  public void lowerElbow() {
    elbow.set(Value.kForward);
  }

  public boolean isWristUp() {
    return wrist.get() == Value.kReverse;
  }

  public boolean isWristDown() {
    return wrist.get() == Value.kForward;
  }

  public void raiseWrist() {
    // make sure to only raise the wrist when elbow is down
    if(isElbowDown()) {
      wrist.set(Value.kReverse);
    } else {
      System.out.println("ERROR: INVALID REQUEST to raise cargo wrist, but the elbow is down");
    }
  }

  public void lowerWrist() {
    wrist.set(Value.kForward);
  }

  /**
   * Use 1 for intake, -1 for eject, 0 to stop motor
   * @param speed
   */
  public void setIntakeRoller(double speed) {
    // once you let go of the button it turns off
    intakeRoller.set(speed);
  }

}
