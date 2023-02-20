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
    TalonSRX linearController;

    // TODO Separate shoulderController into left & right
    // TODO Verify Talons - not necesary because we'll have encoders
    TalonSRX shoulderController;

    // TODO Add Encoder for left and right shoulder

    AnalogInput elbowExtension;

    DigitalInput gripperClosed;
    DigitalInput gripperOpen;

    public ArmSubsystem() {

        gripperController = new TalonSRX(Constants.GRIPPER_CAN);
        linearController = new TalonSRX(Constants.LINEAR_LEFT);
        shoulderController = new TalonSRX(Constants.SHOULDER_CAN);

        // TODO: Configure the continuous current limit for the gripper motor.
        // Hint: Goto the Andy Mark Snow blower motor product page and configure the
        // continuous limit to 1/2 the stall current. (as a starting value)

        // TODO: Configure the continuous current limit for the shoulder controller
        // Hint: Find the motor we plan on using and do the same process.

        // NOTE: Current limiting is NOT required on the linear actuator because it has
        // built in limit switches that prevent stalling the motor

        elbowExtension = new AnalogInput(Constants.LINEAR_ALG);

        gripperClosed = new DigitalInput(Constants.GRIPPER_LIMIT_CLOSED);
        gripperOpen = new DigitalInput(Constants.GRIPPER_LIMIT_OPEN);
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

        linearController.set(TalonSRXControlMode.PercentOutput, elbow);
        shoulderController.set(TalonSRXControlMode.PercentOutput, shoulder);
        gripperController.set(TalonSRXControlMode.PercentOutput, gripper);
    }

    public void armPositionControl(Constants.GripperPosition gripperPosition,
            Constants.ShoulderPosition shoulderPosition, Constants.ElbowPosition elbowPosition) {

        // TODO write function to set to a position
    }

    /**
     * Set the percent output on each shoulder using a single input level. Account
     * for differences in motor efficiency, friction, etc.. By verifying the
     * position of each shoulder are the same.
     * 
     * @param level
     */
    private void setShoulderLevels(double level) {

        // TODO Write an algorithm that uses the input level, and determines a correct
        // levels for each shoulder such that they stay in sync.

    }

    @Override
    public void initSendable(SendableBuilder builder) {

        // TODO Add a shuffleboard variable that displays the difference between the
        // left and right shoulder positions, in millimeters.
    }
}
