// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * One of three possible solutions currently on the table for braking:
 * 1. Use a braking system that deploys mechanically
 * 2. Monitor the encoders and apply small voltage in the opposite direction to
 * hold the robot
 * in place.
 * 3. Pulse the motors forward and backward at a high rate but low level
 */
public class Brake extends CommandBase {

    DriveSubsystem driveSubsystem;

    // orgigionally 10
    static final double SAMPLES_PER_SINE_PERIOD = 5;
    static final double SINE_SAMPLE_PERIOD = (2 * Math.PI) / SAMPLES_PER_SINE_PERIOD;
    double currentSampleRads = 0;

    static final int TICKS_PER_SQUARE_PULSE = 5;
    int ticks = 0;
    double MOTOR_LEVEL = 0.2; // How much motor to apply with each pulse

    public Brake(DriveSubsystem drive) {
        driveSubsystem = drive;
        // addRequirements(drive);
        // if you are using brake do not use the joysticks since the command will battle
        // it and the auto brake
    }

    /**************************************************************************/

    @Override
    public void execute() {
        // executeSquare();
        // executeSine();
        executeImplode();
    }

    /**
     * Make the right mecanum wheels push opposite directions,
     * causing it to remain in place, fighting each other.
     */
    public void executeImplode() {

        // Drive front wheels backward and backwheels forward. This should cause the
        // right wheels to try and push right and the left wheels to push left
        driveSubsystem.runWheel(Constants.FRONT_LEFT_CAN, -MOTOR_LEVEL);
        driveSubsystem.runWheel(Constants.FRONT_RIGHT_CAN, -MOTOR_LEVEL);
        driveSubsystem.runWheel(Constants.BACK_LEFT_CAN, MOTOR_LEVEL);
        driveSubsystem.runWheel(Constants.BACK_RIGHT_CAN, MOTOR_LEVEL);
    }

    /**
     * Rapidly change the motor direction.
     */
    public void executeSquare() {
        ticks++;
        if (ticks % TICKS_PER_SQUARE_PULSE == 0) {
            MOTOR_LEVEL = -MOTOR_LEVEL;
            driveSubsystem.robotDrive(MOTOR_LEVEL, 0, 0);
        }
    }

    public void executeSine() {

        // output frequency = 20 kh (cycles per second)
        //
        // period = 0.00005 seconds per cycle = .05 ms per cycle
        // 3. Assume we want to change the motor direction every 1/10 th of a second.
        // Assume this execute() method is called every 20ms.
        //
        // 4. Calculate the number of times
        // execute() should be called between periods of the sine wave
        // 10

        currentSampleRads = currentSampleRads + SINE_SAMPLE_PERIOD;

        // Origional value of ^ at 0.5 didnt even move
        double motorInputLevel = Math.sin(currentSampleRads) * MOTOR_LEVEL;

        driveSubsystem.robotDrive(motorInputLevel, 0, 0);
    }

    /**************************************************************************/

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    /**
     * This command never finishes.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
