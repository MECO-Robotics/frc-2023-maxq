// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 
 */
public class ClimbingSubsystem extends SubsystemBase {

  private final Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);   // expand
  private final Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);   // contract
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  
  /** Creates a new subsystem. */
  public ClimbingSubsystem() {
    // Automatically turn on compressor when pressure switch indicates pressure is too low and off when pressure is too high
    compressor.enableDigital();

    addChild("Compressor", compressor);
    addChild("Double Solenoid", doubleSolenoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set1(boolean on) {
    solenoid1.set(on);
  }

  public void set2(boolean on) {
    solenoid2.set(on);
  }

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
