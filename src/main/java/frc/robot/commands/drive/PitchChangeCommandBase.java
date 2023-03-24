// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.LinkedList;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This is a base command class, which means it's not intended to be used
 * directly, but instead should be extended by another command class.
 */
public class PitchChangeCommandBase extends CommandBase {

    // The minimum change in pitch required to consider the pitch changed. This
    // accounts for the noise in the IMU plus a little more
    private final double MINIMUM_PITCH_DELTA = 2.5;

    // The pitch under which we consider ourselves level
    private final double NEAR_LEVEL_PITCH = 3.0;

    // The number of historical pitch samples we're keeping. Expect we're storing
    // one every 20ms.
    private final int MAX_PItCH_SAMPLES = 50;

    // A linked list is a data structure that stores a list of items (in this case
    // doubles) that is very fast to add and remove things from the front or back of
    // the list
    private final LinkedList<Double> pitchList = new LinkedList<Double>();

    private final DriveSubsystem drive;

    public PitchChangeCommandBase(DriveSubsystem driveSubsystem) {
        drive = driveSubsystem;
    }

    /**
     * Record a pitch sample in the array
     */
    protected void addPitchSample() {

        // The most recent sample will be at the end of the list
        pitchList.addLast(drive.getPitch());

        if (pitchList.size() >= MAX_PItCH_SAMPLES) {
            // And the oldest sample will be at the front. Remove it if our list has reached
            // it's max size.
            pitchList.removeFirst();
        }
    }

    /**
     * Get if the absolute pitch is decreasing
     * 
     * @return true if the absolute value of the pitch is decreasing
     */
    protected boolean isAbsPitchDecreasing() {

        // If we don't have at least 100ms of data, just return false.
        if (pitchList.size() < 5) {
            return false;
        }

        double previousAbsPitch = Math.abs(pitchList.getFirst());
        double currentAbsPitch = Math.abs(pitchList.getLast());

        return currentAbsPitch < NEAR_LEVEL_PITCH || currentAbsPitch < (previousAbsPitch - MINIMUM_PITCH_DELTA);
    }

    /**
     * Get if the absolute pitch is increasing
     * 
     * @return true if the absolute value of the pitch is increasing
     */
    protected boolean isAbsPitchIncreasing() {

        // If we don't have at least 100ms of data, just return false.
        if (pitchList.size() < 5) {
            return false;
        }

        double previousAbsPitch = Math.abs(pitchList.getFirst());
        double currentAbsPitch = Math.abs(pitchList.getLast());

        return currentAbsPitch > NEAR_LEVEL_PITCH || currentAbsPitch > (previousAbsPitch + MINIMUM_PITCH_DELTA);
    }

}
