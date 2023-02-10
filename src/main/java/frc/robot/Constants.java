// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum GamePiece {
        Unknown,
        Cone,
        Cube
    };

    public enum GripperPosition {
        GripOpen,
        GripClosedCube,
        GripClosedCone
    };

    public enum ShoulderPosition {
        allFoward,
        allBack,
        middle
    };

    public enum ElbowPosition {
        allOut,
        allIn,
        middle

    };

    //
    // DRIVE SUBSYSTEM
    //

    // Mecanum motor controllers
    public static final int FRONT_LEFT_CAN = 2;
    public static final int FRONT_RIGHT_CAN = 3;
    public static final int BACK_LEFT_CAN = 4;
    public static final int BACK_RIGHT_CAN = 5;

    // Arm subsystem motors
    public static final int GRIPPER_CAN = 6;
    public static final int LINEAR_LEFT_CAN = 7;
    public static final int LINEAR_RIGHT_CAN = 8;
    public static final int SHOULDER_CAN = 9;

    // Limit switch DIO Values
    public static final int GRIPPER_LIMIT_OPEN = 0;
    public static final int GRIPPER_LIMIT_CLOSED = 1;
    public static final int SHOULDER_LIMIT_FORWARD = 2;
    public static final int SHOULDER_LIMIT_BACKWARD = 3;

    // Analog Inputs for Elbow Extension Linear Actuator Potentiometer
    public static final int LINEAR_LEFT_ALG = 0;
    public static final int LINEAR_RIGHT_ALG = 1;

    public static final double HIGH_CURRENT_THRESHOLD = 20.0;

    /**
     * How fast we should travel for autonomous commands.
     */
    public static final double AUTO_SPEED = .5;
}
