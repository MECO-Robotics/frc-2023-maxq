// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //
    //          DRIVE SUBSYSTEM
    //

    // Victor SPX CAN Values
    public static final int LEFT_DRIVE_1_CAN = 0;
    public static final int LEFT_DRIVE_2_CAN = 1;
    public static final int LEFT_DRIVE_3_CAN = 2;
    public static final int RIGHT_DRIVE_1_CAN = 3;
    public static final int RIGHT_DRIVE_2_CAN = 4;
    public static final int RIGHT_DRIVE_3_CAN = 5;

    // DIO Values for encoders - just indicate A channel. Assume B channel will be the next port
    public static final int FRONT_LEFT_ENCODER_A_DIO = 0;
    public static final int FRONT_RIGHT_ENCODER_A_DIO = 2;
    public static final int BACK_LEFT_ENCODER_A_DIO = 4;
    public static final int BACK_RIGH_ENCODER_A_DIO = 6;

    // Analog Inputs
    public static final int LEFT_ULTRASONIC_ANLG = 0;
    public static final int MID_ULTRASONIC_ANLG = 1;
    public static final int RIGHT_ULTRASONIC_ANLG = 2;

    // Minimum time required to go zero to full speed, in seconds
    public static final double DEFAULT_MAX_DEMAND_CHANGE = 0.250;

    //
    //          CLIMBING SUBSYSTEM
    //

    // Arm joint pneumatic cylinder - Pneumatic Control Module (PCM) CAN Values
    public static final int ROTATING_ARM_DOUBLE_SOLENOID_FWD_PCM = 1;
    public static final int ROTATING_ARM_DOUBLE_SOLENOID_REV_PCM = 0;
    public static final int PRESSURE_SENSOR_ANLG = 3;

    // Winch motors - Talon SRX CAN Values
    public static final int TELESCOPING_LEFT_WINCH_CAN = 0;
    public static final int TELESCOPING_RIGHT_WINCH_CAN = 1;
    public static final int ROTATING_LEFT_WINCH_CAN = 2;
    public static final int ROTATING_RIGHT_WINCH_CAN = 3;

    // Sign change for commanding motor direction. 
    // Needs to be a double so we can safely multiply 
    // by other doubles and not loose precision
    public static final double TELESCOPING_DIR = -1;
    public static final double ROTATING_DIR = -1;

    public static final double HIGH_CURRENT_THRESHOLD = 20.0;

    // The number of ticks required to fully unwind or wind the telescoping arm winch
    public static final double TELESCOPING_ARM_WINCH_LENGTH_TICKS = 1000;

    // The number of ticks required to fully unwind or wind the telescoping arm winch
    public static final double ROTATING_ARM_WINCH_LENGTH_TICKS = 1000;


    //
    //          CARGO MANIPULATOR SUBSYSTEM
    //

    // PWM Values
    public static final int INTAKE_ROLLER_CAN = 21;


    //pnuematics

    public static final int FORWARD_CHANNEL_WRIST_PCM = 2;
    public static final int BACKWARD_CHANNEL_WRIST_PCM = 3;

    public static final int FORWARD_CHANNEL_ELBOW_PCM = 4;
    public static final int BACKWARD_CHANNEL_ELBOW_PCM = 5;



    /**
     * How fast we should travel for autonomous commands.
     */
    public static final double AUTO_SPEED = .5;
}
