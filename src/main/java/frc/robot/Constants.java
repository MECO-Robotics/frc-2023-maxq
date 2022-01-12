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

    // CAN Values
    public static final int RIGHT_DRIVE_1 = 1;
    public static final int RIGHT_DRIVE_2 = 2;
    public static final int LEFT_DRIVE_1 = 3;
    public static final int LEFT_DRIVE_2 = 4;

    // PWM Values
    public static final int ARM_LIFT = 0;
    public static final int LIFT_EXTENDER = 2;
    public static final int INTAKE = 1;

    // DIO Values
    public static final int RIGHT_ENCODER_1 = 0;
    public static final int RIGHT_ENCODER_2 = 1;
    public static final int LEFT_ENCODER_1 = 2;
    public static final int LEFT_ENCODER_2 = 3;

    public static final int TOP_LIMIT_SWITCH = 4;
    public static final int BOTTOM_LIMIT_SWITCH = 5;

    /**
     * How fast we should travel for autonomous commands.
     */
    public static final double AUTO_SPEED = 0.5;
}
