// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the pneumatically deployed brake shoes that hold the robot in
 * position.
 */
public class PneumaticsSubsystem extends SubsystemBase {

    DoubleSolenoid brakes = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.BRAKES_EXTEND,
            Constants.BRAKES_CONTRACT);

    public void lowerBrakes() {
        // TODO call the method on the brakes object to extend the brakes
    }

    public void raiseBrakes() {
        // TODO call the method on the brakes object to contract the brakes
    }

    // TODO write functions to control intake

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
