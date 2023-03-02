// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the pneumatically deployed brake shoes that hold the robot in
 * position.
 */
public class BrakesSubsystem extends SubsystemBase {

    DoubleSolenoid leftBrakes = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.BRAKES_EXTEND_LEFT_PCM,
            Constants.BRAKES_CONTRACT_LEFT_PCM);

    DoubleSolenoid rightBrakes = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.BRAKES_EXTEND_RIGHT_PCM,
            Constants.BRAKES_CONTRACT_RIGHT_PCM);

    public void lowerBrakes() {
        leftBrakes.set(Value.kForward);
        rightBrakes.set(Value.kForward);
    }

    public void raiseBrakes() {
        leftBrakes.set(Value.kReverse);
        rightBrakes.set(Value.kReverse);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
