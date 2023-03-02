// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    // we are multiplying the difference between the left and right encoders 
    //by this constant to apply a motor speed delta to each shoulder motor.
    final static double MOTOR_ERROR_CONVERSION = .01;
    
    // Gripper
    VictorSP gripperController;
    Constants.GripperPosition desiredGripperPosition;

    // Elbow
    TalonSRX elbowLinearController;
    AnalogInput elbowExtension;
    Constants.ElbowPosition desiredElbowPosition;

    // Shoulder
    TalonSRX leftShoulderController;
    TalonSRX rightShoulderController;
    Encoder leftShoulderEncoder;
    Encoder rightShoulderEncoder;
    Constants.ShoulderPosition desiredShoulderPosition;
    

    public ArmSubsystem() {

        gripperController = new VictorSP(Constants.GRIPPER_PWM);
        elbowLinearController = new TalonSRX(Constants.LINEAR_CAN);
        elbowExtension = new AnalogInput(Constants.LINEAR_ALG);
        leftShoulderController = new TalonSRX(Constants.LEFT_SHOULDER_PWM);
        rightShoulderController = new TalonSRX(Constants.RIGHT_SHOULDER_PWM);

        leftShoulderEncoder = new Encoder(Constants.LEFT_SHOULDER_ENC_A_DIO, Constants.LEFT_SHOULDER_ENC_B_DIO);
        rightShoulderEncoder = new Encoder(Constants.RIGHT_SHOULDER_ENC_A_DIO, Constants.RIGHT_SHOULDER_ENC_B_DIO);
    }

    // This variable creates a timer for the "auger" motor that turns on the gripper
    // motor for a set amount of time (0.85 seconds allows the motor to fully open
    // from a fully closed state & vice versa, with a bit of leniency just to make
    // sure)

    double gripperStartTime;

    // 0.3 of a second to get to full power on linear actuator
    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(3.3);
    // 0.5 of a second to get to full power on sholder motors
    SlewRateLimiter shoulderRateLimiter = new SlewRateLimiter(2);

    @Override
    public void periodic() {
        if ((Timer.getFPGATimestamp() - gripperStartTime) > 0.85) {

        }

        if (desiredGripperPosition == Constants.GripperPosition.GripOpen && gripperController.get() <= 0) {
            openGripper();
        } else if (desiredGripperPosition == Constants.GripperPosition.GripClose && gripperController.get() >= 0) {
            closeGripper();
        }

        if (desiredShoulderPosition == Constants.ShoulderPosition.allForward && leftShoulderController.getMotorOutputPercent() <= 0) {
            setShoulderLevels(shoulderRateLimiter.calculate(0.5));
        } else if (desiredShoulderPosition == Constants.ShoulderPosition.allBack && leftShoulderController.getMotorOutputPercent() >= 0) {
            setShoulderLevels(shoulderRateLimiter.calculate(-0.5));
        }

        if (desiredElbowPosition == Constants.ElbowPosition.allOut && elbowLinearController.getMotorOutputPercent() <= 0) {
            elbowLinearController.set(TalonSRXControlMode.PercentOutput, linearRateLimiter.calculate(1));
        } else if (desiredElbowPosition == Constants.ElbowPosition.allIn && elbowLinearController.getMotorOutputPercent() >= 0) {
            elbowLinearController.set(TalonSRXControlMode.PercentOutput, linearRateLimiter.calculate(-1));
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
    // 8. position to pickup from floor 

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

        elbowLinearController.set(TalonSRXControlMode.PercentOutput, elbow);

        setShoulderLevels(shoulder);

        gripperController.set(gripper);
    }

    

    /**
     * Uses automatic ramping to control arm segments
     * 
     * @param gripperPositionIn  Where we want the gripper to be
     * @param shoulderPositionIn Where we want the shoulder to be
     * @param elbowPositionIn    Where we want the elbow to be
     */
    public void armPositionControl(Constants.GripperPosition gripperPositionIn,
            Constants.ShoulderPosition shoulderPositionIn, Constants.ElbowPosition elbowPositionIn) {

        desiredGripperPosition = gripperPositionIn;
        desiredShoulderPosition = shoulderPositionIn;
        desiredElbowPosition = elbowPositionIn;
    }


    /**
     * Set the percent output on each shoulder using a single input level. Account
     * for differences in motor efficiency, friction, etc.. By verifying the
     * position of each shoulder are the same.
     * 
     * @param level
     */
    private void setShoulderLevels(double level) {

        double deltaArmMotor = (rightShoulderEncoder.get() - leftShoulderEncoder.get()) * MOTOR_ERROR_CONVERSION;

        rightShoulderController.set(ControlMode.PercentOutput, level - deltaArmMotor);
        leftShoulderController.set(ControlMode.PercentOutput, level + deltaArmMotor);

        /*
         * right encoder value subtracted from left to find disparity
         * divided by constant (100) to turn into motor value
         * subtracted from right motor value, added to left motor value
         * if right is greater than left, delta is positive value
         * right motor moves slower, left motor moves faster
         * if left is greater than right, delta is negative value
         * right motor moves faster, left motor moves slower
         */

    }

    public double getShoulderEncoderDelta() {
        return (rightShoulderEncoder.get() - leftShoulderEncoder.get());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Shoulder Encoder Delta", this::getShoulderEncoderDelta, null);
    }
}
