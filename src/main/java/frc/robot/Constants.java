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
        NoChange,
        GripOpen,
        GripClose
    };

    public enum ShoulderPosition {
        NoChange,
        allBackStow,         // 0.0 in position
        middle_MiddleNode,   // 3.0 in position
        middle_LowNode,      // 4.5 in position
        allForward_HighNode  // 7.0 in position
, middle_HighNode
    };
 
    public enum ElbowPosition {
        NoChange,
        allOut, 
        allIn,
        middle_MiddleNode,  // 2.3 in extended
        middle_HighNode,    // 3.1 in extended
        middle_LowNode      // 8.0 in extended
    };

    //
    // DRIVE SUBSYSTEM
    //

    // Mecanum motor controllers
    public static final int FRONT_LEFT_CAN = 2;
    public static final int FRONT_RIGHT_CAN = 3;
    public static final int BACK_LEFT_CAN = 5;
    public static final int BACK_RIGHT_CAN = 4;

    // Arm gripper motors
    public static final int GRIPPER_PWM = 9;
    

    
    // Arm elbow motors
    public static final int LINEAR_CAN_LEFT = 7;
    public static final int LINEAR_CAN_RIGHT =9;

    // Analog Inputs for Elbow Extension Linear Actuator Potentiometer
    public static final int LINEAR_ALG = 0;

    // Arm shoulder motors
    public static final int LEFT_SHOULDER_CAN = 20;
    public static final int RIGHT_SHOULDER_CAN = 21;
    public static final int LEFT_SHOULDER_ENC_A_DIO = 0;
    public static final int LEFT_SHOULDER_ENC_B_DIO = 1;
    public static final int RIGHT_SHOULDER_ENC_A_DIO = 2;
    public static final int RIGHT_SHOULDER_ENC_B_DIO = 3;

    // shoulder limit switches
    public static final int LEFT_SHOULDER_FRONT_LIMIT_DIO = 4;
    public static final int RIGHT_SHOULDER_FRONT_LIMIT_DIO = 5;
    public static final int LEFT_SHOULDER_BACK_LIMIT_DIO = 6;
    public static final int RIGHT_SHOULDER_BACK_LIMIT_DIO = 7;



    // Common motor limits
    public static final double HIGH_CURRENT_THRESHOLD = 20.0;

    public static final int LIGHTS_R_PWM = 3;
    public static final int LIGHTS_G_PWM = 4;
    public static final int LIGHTS_B_PWM = 5;
    
    /**
     * How fast we should travel for autonomous commands.
     */
    public static final double AUTO_SPEED = .5;
    // Feet per second
    public static final double FPS_TO_MPS = 0.308;


    // 
    // Pneumatics subsystem
    //

    public static final int BRAKES_EXTEND_LEFT_PCM = 5;
    public static final int BRAKES_CONTRACT_LEFT_PCM = 4;
    public static final int BRAKES_EXTEND_RIGHT_PCM = 2;
    public static final int BRAKES_CONTRACT_RIGHT_PCM = 3;
    public static final int INTAKE_OUT = 0;
    public static final int INTAKE_IN = 1;

    //
    // Intake subsystem
    //

    public static final int INTAKE_EXTEND_PCM = 6;
    public static final int INTAKE_CONTRACT_PCM = 7;
    public static final int INTAKE_CONTROLLER_CAN = 8;



    //
    // Power
    //

    public static final int LINEAR_POWER_CHANNEL = 0;
    

}
