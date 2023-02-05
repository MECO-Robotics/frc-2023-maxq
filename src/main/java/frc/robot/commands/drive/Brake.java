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
    double executeTime = 10;
    double sinePeriod = (2 * Math.PI) / executeTime;
    double currentSampleRads = 0;

    public Brake(DriveSubsystem drive) {
        driveSubsystem = drive;
        //addRequirements(drive);
        //if you are using brake do not use the joysticks since the command will battle it and the auto brake
    }

    public void execute() {

        // output frequency = 20 kh (cycles per second)
        //
        // period = 0.00005 seconds per cycle = .05 ms per cycle
        // 3. Assume we want to change the motor direction every 1/10 th of a second.
        // Assume this execute() method is called every 20ms.
        //
        // 4. Calculate the number of times
        // execute() should be called between periods of the sine wave
        // 10

        currentSampleRads = currentSampleRads + sinePeriod;

        double motorInputLevel = Math.sin(currentSampleRads);
        motorInputLevel = motorInputLevel * 0.05;

        driveSubsystem.robotDrive(motorInputLevel, 0, 0);

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
