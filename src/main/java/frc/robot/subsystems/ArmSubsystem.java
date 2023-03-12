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

public class ArmSubsystem extends SubsystemBase {

    // Simple counter that reduces the number of times something is printe to the
    // console
    long logger = 0;

    /**
     * Time to run the gripper motor open or closed at 100%.
     */
    private static final double GRIPPER_MOVE_TIME_SEC = 0.85;

    private static final int LINEAR_MAX = 1100;
    private static final int LINEAR_MIN = 370;
    private static final int TPI = 80; // Ticks per inch of string pot

    // max 1189
    // min 289

    // we are multiplying the difference between the left and right encoders
    // by this constant to apply a motor speed delta to each shoulder motor.
    final static double MOTOR_ERROR_CONVERSION = .2;

    // Gripper
    VictorSP gripperController;
    Constants.GripperPosition desiredGripperPosition = Constants.GripperPosition.NoChange;
    double gripperStartTime;

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
    Constants.ShoulderPosition desiredShoulderPosition = Constants.ShoulderPosition.NoChange;

    // 0.5 of a second to get to full power on sholder motors
    SlewRateLimiter shoulderRateLimiter = new SlewRateLimiter(2);

    DigitalInput shoulderLeftFrontLimit = new DigitalInput(Constants.LEFT_SHOULDER_FRONT_LIMIT_DIO);
    DigitalInput shoulderLeftBackLimit = new DigitalInput(Constants.LEFT_SHOULDER_BACK_LIMIT_DIO);
    DigitalInput shoulderRightFrontLimit = new DigitalInput(Constants.RIGHT_SHOULDER_FRONT_LIMIT_DIO);
    DigitalInput shoulderRightBackLimit = new DigitalInput(Constants.RIGHT_SHOULDER_BACK_LIMIT_DIO);

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
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {

        //
        // GRIPPER
        //

        // No matter what, if the motor has been running more than the max amount of
        // time, then stop the motor.
        if ((Timer.getFPGATimestamp() - gripperStartTime) > GRIPPER_MOVE_TIME_SEC) {
            gripperController.set(0);
            gripperStartTime = 0;

        } else if (desiredGripperPosition == Constants.GripperPosition.GripOpen && gripperController.get() < 0.0001) {
            openGripper();

        } else if (desiredGripperPosition == Constants.GripperPosition.GripClose && gripperController.get() > 0.0001) {
            closeGripper();
        }

        //
        // SHOULDER
        //

        if (desiredShoulderPosition == Constants.ShoulderPosition.allForward_HighNode
                && leftShoulderController.getMotorOutputPercent() <= 0) {

            setShoulderLevels(shoulderRateLimiter.calculate(0.5));

        } else if (desiredShoulderPosition == Constants.ShoulderPosition.allBackStow
                && leftShoulderController.getMotorOutputPercent() >= 0) {

            setShoulderLevels(shoulderRateLimiter.calculate(-0.5));
        }

        SmartDashboard.putNumber("Left Shoulder", getLeftShoulderPosition());
        SmartDashboard.putNumber("Right Shoulder", getRightShoulderPosition());
    }

    // -------------------------------------------------------------------
    // GRIPPER
    //

    public void move(Constants.GripperPosition gripperPositionIn) {
        // Real work is done in periodic. This just sets the goal state
        desiredGripperPosition = gripperPositionIn;
    }

    /**
     * IMPORTANT: Only call this once, not repeatedly!
     */
    public void openGripper() {
        gripperStartTime = Timer.getFPGATimestamp();
        gripperController.set(1.0);
    }

    /**
     * IMPORTANT: Only call this once, not repeatedly!
     */
    public void closeGripper() {
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
            case middle_MiddleNode: // 2.3 in extended
                return 2.3 * TPI;
            case middle_HighNode: // 3.1 in extended
                return 3.1 * TPI;
            case middle_LowNode: // 8.0 in extended
                return 3.1 * TPI;
        }

        return 0;
    }

    // -------------------------------------------------------------------
    // SHOULDER
    //

    public void move(Constants.ShoulderPosition shoulderPositionIn) {
        System.out.println("moving shoulder");
        desiredShoulderPosition = shoulderPositionIn;
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

        boolean frontOKL = !shoulderLeftFrontLimit.get() && leftLevel < 0;
        boolean backOKL = !shoulderLeftBackLimit.get() && leftLevel > 0;
        boolean midOKL = shoulderLeftBackLimit.get() && shoulderLeftFrontLimit.get();
        boolean frontOKR = !shoulderRightFrontLimit.get() && rightLevel < 0;
        boolean backOKR = !shoulderRightBackLimit.get() && rightLevel > 0;
        boolean midOKR = shoulderRightBackLimit.get() && shoulderRightFrontLimit.get();

        if (!shoulderLeftFrontLimit.get() || !shoulderRightFrontLimit.get()) {
            return;
        }

        if (!shoulderLeftBackLimit.get() || !shoulderRightBackLimit.get()) {
            return;
        }

        if (frontOKL || backOKL || midOKL) {
            leftShoulderController.set(ControlMode.PercentOutput, leftLevel);
        }

        if (frontOKR || backOKR || midOKR) {
            rightShoulderController.set(ControlMode.PercentOutput, rightLevel);
        }

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

        if (logger++ % 50 == 0)
            System.out.println(String.format("ArmSubsystem.manualControl: %f, %f, %f", elbow, shoulder, gripper));

        elbowLinearControllerLeft.set(TalonSRXControlMode.PercentOutput, elbow);
        elbowLinearControllerRight.set(TalonSRXControlMode.PercentOutput, elbow);

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
