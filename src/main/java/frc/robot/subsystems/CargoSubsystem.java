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
 * 1 motor for ball launch
 * Sensors
 * 1 Encoder for ball launch
 */
public class CargoSubsystem extends SubsystemBase {

  Spark intakeRoller = new Spark(Constants.INTAKE_ROLLER_PWM_CHANNEL);
  DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_WRIST,
      Constants.BACKWARD_CHANNEL_WRIST);
  DoubleSolenoid elbow = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_ELBOW,
      Constants.BACKWARD_CHANNEL_ELBOW);

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

  public void raiseCargoElbow() {
    // make sure that the wirst is down before raising the elbow
    if (wrist.get() == Value.kForward) {

    } else {
      wrist.set(Value.kReverse);
    }

    elbow.set(Value.kForward);

  }

  public void lowerCargoElbow() {
    elbow.set(Value.kReverse);
  }

  public void raiseCargoWrist() {
    // make sure not to raise the wrist when elbow is up
    wrist.set(Value.kForward);

  }

  public void lowerCargoWrist() {
    wrist.set(Value.kReverse);
  }

  public void intakeRoller(double speed) {
    // once you let go of the button it turns off
    intakeRoller.set(speed);

  }

}
