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
    PIDController shoulderPid = new PIDController(.9, 0, 0, 0.020); // 20ms period
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
        elbowPid.setTolerance(10); // Ticks

        leftShoulderController = new TalonSRX(Constants.LEFT_SHOULDER_CAN);
        rightShoulderController = new TalonSRX(Constants.RIGHT_SHOULDER_CAN);
        leftShoulderController.setInverted(true);
        rightShoulderController.setInverted(true);
        shoulderPid.setTolerance(0.1); // Revolutions

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
        switch (gripperPositionIn) {
            case GripOpen:
                openGripper();
                break;
            case GripClose:
                closeGripper();
                break;
            case NoChange:
                break;
        }
    }

    /**
     * IMPORTANT: Only call this once, not repeatedly!
     */
    public void openGripper() {
        gripperButtonControl = true;
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(-1.0);
    }

    /**
     * IMPORTANT: Only call this once, not repeatedly!
     */
    public void closeGripper() {
        gripperButtonControl = true;
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(1.0);
    }

    // -------------------------------------------------------------------
    // ELBOW
    //

    public boolean move(Constants.ElbowPosition elbowPositionIn) {

        double setPoint = getElbowExtension(elbowPositionIn);

        double error = setPoint - elbowExtension.getValue();

        System.out.println(String.format("ELBOW: setPoint:%f, measurement:%d", setPoint, elbowExtension.getValue()));
        if (Math.abs(error) > 10) {
            double motor = error * -0.02;

            // Temporary limit to elbow speed to +/- 0.5
            motor = Math.signum(motor) * Math.min(0.5, Math.abs(motor));

            elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, motor);
            elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, motor);

            return false;
        } else {
            return true;
        }
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
                return 522;
            // 632
            case middle_HighNodeCone:
                return 580;
            // 750
            case middle_HighNodeCube:
                return 660;
            case middle_PickUp:
                return 1323;
            // 1480
            case stow:
                return 1028;
            // 1138
        }

        return 0;
    }

    // -------------------------------------------------------------------
    // SHOULDER
    //

    public void resetEncoders() {
        leftShoulderController.setSelectedSensorPosition(0);
        rightShoulderController.setSelectedSensorPosition(0);
    }

    public boolean move(Constants.ShoulderPosition shoulderPositionIn) {

        double setPoint = getShoulderPos(shoulderPositionIn);

        double measurement = getLeftShoulderPosition();// right is broke

        double level = shoulderPid.calculate(measurement, setPoint);

        boolean atLimit = setShoulderLevels(level);
        
        return atLimit || shoulderPid.atSetpoint();
    }

    private double getShoulderPos(ShoulderPosition pos) {

        switch (pos) {
            case NoChange:
                return 0;
            case allBackStow:
                return 0; // L: -0.806, R: -0.772
            // OLD: 0; L: -0.804, R: -0.818
            case middle_MiddleNode:
                return 0.337; // L: 0.453, R: 0.458
            // OLD: -0.48; L: -0.466, R: -0.498
            case middle_LowNode:
                return 0.384; // L: -0.407, R: -0.382
            // OLD: -0.375; L: -0.379, R: -0.367
            case middle_HighNode: // 3.1 in extended
                return 0.955; // all forward (L: 0.150, R: 0.197)
            // OLD: 0.2; all forward (L: 0.131, R: 0.136)
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
    private boolean setShoulderLevels(double level) {

        double deltaArmMotor = getShoulderEncoderDelta() * MOTOR_ERROR_CONVERSION;
        deltaArmMotor = 0;
        double rightLevel = level - deltaArmMotor;
        double leftLevel = level + deltaArmMotor;

        if (!shoulderLeftFrontLimit.get() || !shoulderRightFrontLimit.get()) {
            if (rightLevel > 0 || leftLevel > 0) {
                // Still trying to go forward, but limits are hit, so stop and return true,
                // which means we're done
                leftShoulderController.set(ControlMode.PercentOutput, 0);
                rightShoulderController.set(ControlMode.PercentOutput, 0);
                return true;
            }
        }

        if (!shoulderLeftBackLimit.get() || !shoulderRightBackLimit.get()) {
            resetEncoders();
            if (rightLevel < 0 || leftLevel < 0) {
                // Still trying to go backwards, but we've reached the back limits
                leftShoulderController.set(ControlMode.PercentOutput, 0);
                rightShoulderController.set(ControlMode.PercentOutput, 0);
                return true;
            }
        }

        leftShoulderController.set(ControlMode.PercentOutput, leftLevel);
        rightShoulderController.set(ControlMode.PercentOutput, rightLevel);

        return false;
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

            // if (useHoldPosition) {
            // positionDrift = holdElbowPosition - elbowExtension.getValue();

            // if (Math.abs(positionDrift) > 40) {
            // double elbowLevel = Math.signum(positionDrift) * 0.2;
            // elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, elbowLevel);
            // elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput,
            // elbowLevel);
            // }

            // } else {
            // useHoldPosition = true;
            // holdElbowPosition = elbowExtension.getValue();
            // }

            // } else {
            // useHoldPosition = false;
            elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, elbow);
            elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, elbow);
            // }

            // if (logger % 10 == 0){
            // System.out
            // .println((useHoldPosition ? "HOLDING" : "MOVING") + String.format("; DRIFT:
            // %d", positionDrift));
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
        builder.addBooleanProperty("RFL", () -> {
            return shoulderRightFrontLimit.get();
        }, null);
        builder.addBooleanProperty("LFL", () -> {
            return shoulderLeftFrontLimit.get();
        }, null);
        builder.addBooleanProperty("RBL", () -> {
            return shoulderRightBackLimit.get();
        }, null);
        builder.addBooleanProperty("LBL", () -> {
            return shoulderLeftBackLimit.get();
        }, null);
    }
}
