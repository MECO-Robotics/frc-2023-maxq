// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        gripperController = new TalonSRX(Constants.GRIPPER_CAN);
        linearControllerLeft = new TalonSRX(Constants.LINEAR_LEFT_CAN);
        linearControllerRight = new TalonSRX(Constants.LINEAR_RIGHT_CAN);
        shoulderController = new TalonSRX(Constants.SHOULDER_CAN);

        elbowExtensionLeft = new AnalogInput(Constants.LINEAR_LEFT_ALG);
        elbowExtensionRight = new AnalogInput(Constants.LINEAR_RIGHT_ALG);

        gripperClosed = new DigitalInput(Constants.GRIPPER_LIMIT_CLOSED);
        gripperOpen = new DigitalInput(Constants.GRIPPER_LIMIT_OPEN);
        shoulderForward = new DigitalInput(Constants.SHOULDER_LIMIT_FORWARD);
        shoulderBackward = new DigitalInput(Constants.SHOULDER_LIMIT_BACKWARD);
    }

    @Override
    public void periodic() {
    }

    // TODO Write functions to perform the following:
    // 1. Stow arm - this would fully contract the linear actuators and probably
    // position the shoulder back
    // 2. Top grid - fully extend elbow and shoulder
    // 3. middle grid - partially extend elbow & shoulder
    // 4. low grid - floor placement
    // 5. open / close gripper
    // 6. position to pick up from dual loading station
    // 7. position to pick up from single loading station
    // 8. position to pickup from floow

    /**
     * Move the articulations of the arm. Parameters are the speed the motors should
     * move.
     * 
     * @param elbow    -1 .. 1 - Contract .. Expand linear actuators (1.0 runs to
     *                 extend the elbow)
     * @param shoulder -1 .. 1 - backward .. forward
     * @param gripper  -1 .. 1 - close .. open
     */
    public void manualControl(double elbow, double shoulder, double gripper) {

        linearControllerLeft.set(TalonSRXControlMode.Velocity, elbow);
        linearControllerRight.set(TalonSRXControlMode.Velocity, elbow);
        shoulderController.set(TalonSRXControlMode.Velocity, shoulder);
        gripperController.set(TalonSRXControlMode.Velocity, gripper);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
