// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // TODO Declare variables for 4 motor controllers. Let's assume CAN Motor
    // controllers for now.
    // The gripper and shoulder motor need to be TalonSRX or SparkMax's because they
    // have built-in current limiting, which allows us to automatically kill power
    // when the motor stalls when reaching the end
    // 1. Gripper
    // 2. Linear actuators - left & right - "elbow"
    // 3. The shoulder motor

    // TODO Declare 2 AnalogInput variables that will provide feedback on how far
    // the elbow is extended

    // TODO Declare 2 limit switches - one for gripper and one for the shoulder

    public ArmSubsystem() {

        // TODO Add constants to the Constants.java file for the CAN IDs of the montor
        // controllers

        // Setup the VictorSPX motor controllers

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
