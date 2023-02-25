// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    // Gripper
    VictorSP gripperController;

    // Elbow
    TalonSRX linearController;
    AnalogInput elbowExtension;

    // Shoulder
    VictorSP leftShoulderController;
    VictorSP rightShoulderController;
    Encoder leftShoulderEncoder;
    Encoder rightShoulderEncoder;

    public ArmSubsystem() {

        gripperController = new VictorSP(Constants.GRIPPER_PWM);
        linearController = new TalonSRX(Constants.LINEAR_CAN);
        elbowExtension = new AnalogInput(Constants.LINEAR_ALG);
        leftShoulderController = new VictorSP(Constants.LEFT_SHOULDER_PWM);
        rightShoulderController = new VictorSP(Constants.RIGHT_SHOULDER_PWM);

        leftShoulderEncoder = new Encoder(Constants.LEFT_SHOULDER_ENC_A_DIO, Constants.LEFT_SHOULDER_ENC_B_DIO);
        rightShoulderEncoder = new Encoder(Constants.RIGHT_SHOULDER_ENC_A_DIO, Constants.RIGHT_SHOULDER_ENC_B_DIO);
    }

    // This variable creates a timer for the "auger" motor that turns on the gripper
    // motor for a set amount of time (0.85 seconds allows the motor to fully open
    // from a fully closed state & vice versa, with a bit of leniency just to make
    // sure)

    double gripperStartTime;

    @Override
    public void periodic() {
        if ((Timer.getFPGATimestamp() - gripperStartTime) > 0.85) {
            gripperController.set(0);
        }

    }

    public void openGripper() {
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(1.0);
    }

    public void closeGripper() {
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(-1.0);
    }

    // TODO Write functions to perform the following:
    // 1. Stow arm - this would fully contract the linear actuators and probably
    // position the shoulder back
    // 2. Top grid - fully extend elbow and shoulder
    // 3. middle grid - partially extend elbow & shoulder
    // 4. low grid - floor placement
    // 5. open / close gripper - DONE!
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
        
        setShoulderLevels(shoulder);
        
        gripperController.set(gripper);
    }

    public void armPositionControl(Constants.GripperPosition gripperPosition,
            Constants.ShoulderPosition shoulderPosition, Constants.ElbowPosition elbowPosition) {

        // TODO write function to set to a position
    }

    final static double MOTOR_ERROR_CONVERSION = .01;

    /**
     * Set the percent output on each shoulder using a single input level. Account
     * for differences in motor efficiency, friction, etc.. By verifying the
     * position of each shoulder are the same.
     * 
     * @param level
     */
    private void setShoulderLevels(double level) {

        double deltaArmMotor = (rightShoulderEncoder.get() - leftShoulderEncoder.get()) * MOTOR_ERROR_CONVERSION;

        rightShoulderController.set(level - deltaArmMotor);
        leftShoulderController.set(level + deltaArmMotor);

        /* right encoder value subtracted from left to find disparity
        *  divided by constant (100) to turn into motor value
        *  subtracted from right motor value, added to left motor value
        *    if right is greater than left, delta is positive value
        *    right motor moves slower, left motor mvoes faster
        *    if left is greater than right, delta is negative value
        *    right motor moves faster, left motor mvoes slower
        */
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        // TODO Add a shuffleboard variable that displays the difference between the
        // left and right shoulder positions, in millimeters.
    }
}
