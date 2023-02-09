// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // The gripper and shoulder motor need to be TalonSRX because they
    // have built-in current limiting, which allows us to automatically kill power
    // when the motor stalls when reaching the end

    TalonSRX gripperController;
    TalonSRX linearControllerLeft;
    TalonSRX linearControllerRight;
    TalonSRX shoulderController;

    AnalogInput elbowExtensionLeft;
    AnalogInput elbowExtensionRight;

    DigitalInput gripperClosed;
    DigitalInput gripperOpen;
    DigitalInput shoulderForward;
    DigitalInput shoulderBackward;
    
    public ArmSubsystem() {

    }

    @Override
    public void periodic() {
    }

    // TODO Write functions to perform the following:
    // 1. Stow arm -  this would fully contract the linear actuators and probably position the shoulder back
    // 2. Top grid - fully extend elbow and shoulder
    // 3. middle grid - partially extend elbow & shoulder
    // 4. low grid - floor placement
    // 5. open / close gripper
    // 6. position to pick up from dual loading station
    // 7. position to pick up from single loading station
    // 8. position to pickup from floow
    // 9. manual arm control - elbow, shoulder, gripper   <-- LET'S CODE THIS FIRST!!!!
    
    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
