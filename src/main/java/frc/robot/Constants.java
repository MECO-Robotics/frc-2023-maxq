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

    public enum GamePiece {
        Unknown,
        Cone,
        Cube
    };

    //
    //          DRIVE SUBSYSTEM
    //

    // Victor SPX CAN Values
    public static final int FRONT_LEFT_CAN = 1;
    public static final int FRONT_RIGHT_CAN = 2;
    public static final int BACK_LEFT_CAN = 3;
    public static final int BACK_RIGHT_CAN = 4;

    // DIO Values for encoders - just indicate A channel. Assume B channel will be the next port
    public static final int FRONT_LEFT_ENCODER_A_DIO = 0;
    public static final int FRONT_RIGHT_ENCODER_A_DIO = 2;
    public static final int BACK_LEFT_ENCODER_A_DIO = 4;
    public static final int BACK_RIGHT_ENCODER_A_DIO = 6;

    public static final double HIGH_CURRENT_THRESHOLD = 20.0;



    /**
     * How fast we should travel for autonomous commands.
     */
    public static final double AUTO_SPEED = .5;
}
