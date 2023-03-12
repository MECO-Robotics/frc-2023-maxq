// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderPosition;

public class ArmSubsystem extends SubsystemBase {

    // Simple counter that reduces the number of times something is printe to the
    // console
    long logger = 0;

    /**
     * Time to run the gripper motor open or closed at 100%.
     */
    private static final double GRIPPER_MOVE_TIME_SEC = 0.85;

    private static final int LINEAR_MAX = 2000;
    private static final int LINEAR_MIN = 0;

    // max 1189
    // min 289

    // we are multiplying the difference between the left and right encoders
    // by this constant to apply a motor speed delta to each shoulder motor.
    final static double MOTOR_ERROR_CONVERSION = .2;

    // Gripper
    VictorSP gripperController;
    Constants.GripperPosition desiredGripperPosition = Constants.GripperPosition.NoChange;
    double gripperStartTime;
    // true when we're using openGripper and closeGripper, false when using
    // manualControl()
    private boolean gripperButtonControl;

    // Elbow
    TalonSRX elbowLinearControllerLeft;
    TalonSRX elbowLinearControllerRight;

    AnalogInput elbowExtension;
    Constants.ElbowPosition desiredElbowPosition = Constants.ElbowPosition.NoChange;
    // ~80 encoder ticks per inch so within an inch, it should start proportional
    // control. So d = 1/80 = 0.0125;
    PIDController elbowPid = new PIDController(0.0125, 0, 0);
    // 0.3 of a second to get to full power on linear actuator
    SlewRateLimiter elbowRateLimiter = new SlewRateLimiter(3.3);

    // Shoulder
    TalonSRX leftShoulderController;
    TalonSRX rightShoulderController;
    ShoulderPosition desiredShoulderPosition = ShoulderPosition.NoChange;
    PIDController shoulderPid = new PIDController(.1, 0, 0);
    // 0.5 of a second to get to full power on sholder motors
    SlewRateLimiter shoulderRateLimiter = new SlewRateLimiter(2);

    DigitalInput shoulderLeftFrontLimit = new DigitalInput(Constants.LEFT_SHOULDER_FRONT_LIMIT_DIO);
    DigitalInput shoulderLeftBackLimit = new DigitalInput(Constants.LEFT_SHOULDER_BACK_LIMIT_DIO);
    DigitalInput shoulderRightFrontLimit = new DigitalInput(Constants.RIGHT_SHOULDER_FRONT_LIMIT_DIO);
    DigitalInput shoulderRightBackLimit = new DigitalInput(Constants.RIGHT_SHOULDER_BACK_LIMIT_DIO);

    private boolean allowManualControl = true;

    private int holdElbowPosition;

    private boolean useHoldPosition;

    public ArmSubsystem() {

        setName("ARM");

        gripperController = new VictorSP(Constants.GRIPPER_PWM);
        gripperController.setInverted(true);
        elbowLinearControllerLeft = new TalonSRX(Constants.LINEAR_CAN_LEFT);
        elbowLinearControllerRight = new TalonSRX(Constants.LINEAR_CAN_RIGHT);
        elbowExtension = new AnalogInput(Constants.LINEAR_ALG);
        leftShoulderController = new TalonSRX(Constants.LEFT_SHOULDER_CAN);
        rightShoulderController = new TalonSRX(Constants.RIGHT_SHOULDER_CAN);
        leftShoulderController.setInverted(true);
        rightShoulderController.setInverted(true);

        addChild("Elbow Pos", elbowExtension);
        addChild("Elbow PID", elbowPid);
        addChild("shoulderLFL", shoulderLeftFrontLimit);
        addChild("shoulderLBL", shoulderLeftBackLimit);
        addChild("shoulderRFL", shoulderRightFrontLimit);
        addChild("shoulderRBL", shoulderRightBackLimit);
        addChild("Shoulder PID", shoulderPid);
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {

        logger++;
        //
        // GRIPPER
        //

        // No matter what, if the motor has been running more than the max amount of
        // time, then stop the motor.
        if (gripperButtonControl) {
            if ((Timer.getFPGATimestamp() - gripperStartTime) > GRIPPER_MOVE_TIME_SEC) {
                gripperController.set(0);
                gripperStartTime = 0;

            } else if (desiredGripperPosition == Constants.GripperPosition.GripOpen
                    && gripperController.get() < 0.0001) {
                openGripper();

            } else if (desiredGripperPosition == Constants.GripperPosition.GripClose
                    && gripperController.get() > 0.0001) {
                closeGripper();
            }
        }
    }

    // -------------------------------------------------------------------
    // GRIPPER
    //

    public void moveGripper(double level) {
        gripperButtonControl = false;
        gripperController.set(level);
    }

    public void move(Constants.GripperPosition gripperPositionIn) {
        // Real work is done in periodic. This just sets the goal state
        desiredGripperPosition = gripperPositionIn;
    }

    /**
     * IMPORTANT: Only call this once, not repeatedly!
     */
    public void openGripper() {
        gripperButtonControl = true;
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(1.0);
    }

    /**
     * IMPORTANT: Only call this once, not repeatedly!
     */
    public void closeGripper() {
        gripperButtonControl = true;
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(-1.0);
    }

    // -------------------------------------------------------------------
    // ELBOW
    //

    public void move(Constants.ElbowPosition elbowPositionIn) {

        System.out.println("moving elbow");
        double desired = getElbowExtension(elbowPositionIn);

        double motor = elbowPid.calculate(elbowExtension.getValue(), desired);

        motor = elbowRateLimiter.calculate(motor);
        elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, motor);
        elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, motor);
    }

    private double getElbowExtension(Constants.ElbowPosition elbowPos) {
        // Roughly 80 encoder ticks per inch
        switch (elbowPos) {
            case NoChange:
                return 0;
            case allOut:
                return LINEAR_MAX;
            case allIn:
                return LINEAR_MIN;
            case middle_MiddleNode:
                return 632;
            case middle_HighNode:
                return 750;
            case middle_LowNode: // also pick up
                return 1480;
            case stow:
                return 1138;
        }

        return 0;
    }

    // -------------------------------------------------------------------
    // SHOULDER
    //

    public void move(Constants.ShoulderPosition shoulderPositionIn) {
        allowManualControl = false;
        double setPoint = getShoulderPos(shoulderPositionIn);
        double measurement = (getLeftShoulderPosition() + getRightShoulderPosition()) / 2.0;    // average position
        double level = shoulderPid.calculate(measurement, setPoint);
        level = shoulderRateLimiter.calculate(level);
        setShoulderLevels(level);
    }

    private double getShoulderPos(ShoulderPosition pos) {

        switch (pos) {
            case NoChange:
                return 0;
            case allBackStow:
                return 0; // L: -0.804, R: -0.818
            case middle_MiddleNode:
                return -0.48; // L: -0.466, R: -0.498
            case middle_LowNode: // L: -0.379, R: -0.367
                return -0.375;
            case middle_HighNode: // 3.1 in extended
                return 0.2; // all forward (L: 0.131, R: 0.136)
        }

        return 0;
    }

    /**
     * Move the shoulder, ignoring the limit switches
     * 
     * @param level
     */
    private void setShoulderLevelsIgnoreLimits(double level) {

        double deltaArmMotor = getShoulderEncoderDelta() * MOTOR_ERROR_CONVERSION;
        double rightLevel = level - deltaArmMotor;
        double leftLevel = level + deltaArmMotor;

        leftShoulderController.set(ControlMode.PercentOutput, leftLevel);
        rightShoulderController.set(ControlMode.PercentOutput, rightLevel);
    }

    /**
     * Set the percent output on each shoulder using a single input level. Account
     * for differences in motor efficiency, friction, etc.. By verifying the
     * position of each shoulder are the same.
     * 
     * @param level
     */
    private void setShoulderLevels(double level) {

        double deltaArmMotor = getShoulderEncoderDelta() * MOTOR_ERROR_CONVERSION;
        double rightLevel = level - deltaArmMotor;
        double leftLevel = level + deltaArmMotor;

        if (!shoulderLeftFrontLimit.get() || !shoulderRightFrontLimit.get()) {
            if (rightLevel > 0 || leftLevel > 0) {
                return;
            }
        }

        if (!shoulderLeftBackLimit.get() || !shoulderRightBackLimit.get()) {
            if (rightLevel < 0 || leftLevel < 0) {
                return;
            }
        }

        leftShoulderController.set(ControlMode.PercentOutput, leftLevel);
        rightShoulderController.set(ControlMode.PercentOutput, rightLevel);

    }

    /**
     * Get the difference between the right and left motor
     * 
     * @return
     */
    public double getShoulderEncoderDelta() {
        return getRightShoulderPosition() - getLeftShoulderPosition();
    }

    public double getLeftShoulderPosition() {
        if (leftShoulderController == null) {
            return 0.0;
        }
        return (leftShoulderController.getSelectedSensorPosition() / 500000);
    }

    public double getRightShoulderPosition() {
        if (rightShoulderController == null) {
            return 0.0;
        }
        return -(rightShoulderController.getSelectedSensorPosition() / 500000);
    }

    // -------------------------------------------------------------------
    // ALL CONTROL
    //

    public void manualControl(double elbow, double shoulder) {
        if (allowManualControl) {

            // int positionDrift = 0;

            // if (Math.abs(elbow) < 0.05) {

            //     if (useHoldPosition) {
            //         positionDrift = holdElbowPosition - elbowExtension.getValue();

            //         if (Math.abs(positionDrift) > 40) {
            //             double elbowLevel = Math.signum(positionDrift) * 0.2;
            //             elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, elbowLevel);
            //             elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, elbowLevel);
            //         }

            //     } else {
            //         useHoldPosition = true;
            //         holdElbowPosition = elbowExtension.getValue();
            //     }

            // } else {
            //     useHoldPosition = false;
                elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, elbow);
                elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, elbow);
            // }

            // if (logger % 10 == 0){
            //     System.out
            //             .println((useHoldPosition ? "HOLDING" : "MOVING") + String.format("; DRIFT: %d", positionDrift));
            // }

            setShoulderLevels(shoulder);
        }
    }

    /**
     * Move the articulations of the arm. Parameters are the speed the motors should
     * move.
     * 
     * @param elbow    -1 .. 1 - Contract .. Expand linear actuators (1.0 runs to
     *                 extend the elbow)
     * @param shoulder -1 .. 1 - backward .. forward
     * @param gripper  -1 .. 1 - close .. open
     */
    public void manualControlNoLimits(double elbow, double shoulder, double gripper) {

        // if (logger % 50 == 0)
        // System.out.println(String.format("ArmSubsystem.manualControl: %f, %f, %f",
        // elbow, shoulder, gripper));

        elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, elbow);
        elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, elbow);

        setShoulderLevelsIgnoreLimits(shoulder);

        gripperButtonControl = false;
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

    // -------------------------------------------------------------------

    private int getStringPot() {
        return elbowExtension.getValue();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Shoulder Encoder Delta", this::getShoulderEncoderDelta, null);
        builder.addDoubleProperty("Left Shoulder Position", this::getLeftShoulderPosition, null);
        builder.addDoubleProperty("Right Shoulder Position", this::getRightShoulderPosition, null);
        builder.addIntegerProperty("String Pot", this::getStringPot, null);
    }
}
