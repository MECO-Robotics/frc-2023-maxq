// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

/**
 * An arm with a hook at the end that can extend, then contract to lift the robot.
 */
public class ClimbingArmSubsystem extends SubsystemBase {

  Victor liftExtender = new Victor(Constants.LIFT_EXTENDER);

  /** Creates a new subsystem. */
  public ClimbingArmSubsystem() {
    addChild("Lift motor", liftExtender);
  }

  public boolean isArmExtended() {
    return false;
  }

  /**
   * Move the arm with a specified speed. 
   * @param upSpeed Motor speed. -1.0 (down) - 1.0 (up)
   */
  public void moveArm(double upSpeed) {
    liftExtender.set(upSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
