// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
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

    public Brake(DriveSubsystem drive) {
        driveSubsystem = drive;
        addRequirements(drive);
    }

    public void execute() {

        // TODO: Write this command.
        //

        //
        // PART 1 - CALCULATIONS
        //

        // 1. Goto
        // https://www.andymark.com/products/spark-max-brushless-and-brushed-dc-motor-controller
        // and get the output frequency from the specifications.
        //
        // 2. Determine the period from the frequency
        //
        // 3. Assume we want to change the motor direction every 1/10 th of a second.
        // Assume this execute() method is called every 20ms.
        //
        // 4. Calculate the number of times
        // execute() should be called between switching motor directions
        //

        //
        // PART 2 - READY TO CODE
        //

        // 1. Define a constant on this class that is the number of times execute()
        // should be called between changing motor directions

        // 2. Define a constant on this class that is 2 X Pi divided by the constant
        // just defined.

        // 3. Define a variable on this class initialized to zero that is a number of
        // radians

        // 4. Write a line in this function that increments the radians variable by the
        // the constant in step 3.

        // 5. Write a line that takes the sine of the variable. Put the result in a new
        // variable

        // 6. Call the robot drive method on the Drive Subsystem and pass in the new
        // variable in for the forward parameter, and zero for the strafe and twist
        // parameters.
    }

    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    /**
     * This command never finishes.
     */
    public boolean isFinished() {
        return false;
    }
}
