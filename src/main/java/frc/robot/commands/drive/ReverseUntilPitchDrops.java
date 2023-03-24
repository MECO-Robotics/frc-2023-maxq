// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command to auto level on the charge station. See the
 * DriveSubsystem.chargeStationEnergize() method for requirements.
 * Command is finished when the robot is level and stationary
 */
public class ReverseUntilPitchDrops extends CommandBase {

    private final DriveSubsystem drive;
    private final List<Double> pitchList = new ArrayList<Double>();

    public ReverseUntilPitchDrops(DriveSubsystem driveSubsystem) {

        drive = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    /**
     * Get if the absolute pitch is decreasing
     * 
     * @return
     */
    private boolean isAbsPitchDecreasing() {

        pitchList.add(drive.getPitch());

        if (pitchList.size() >= 50) {
            pitchList.remove(0);
        }

        // If we don't have at least 100ms of data, just return false.
        if (pitchList.size() < 5) {
            return false;
        }
        double previousAbsPitch = Math.abs(pitchList.get(0));
        double currentAbsPitch = Math.abs(pitchList.get(pitchList.size() - 1));

        return currentAbsPitch < 3 || currentAbsPitch < (previousAbsPitch - 2.5);
    }

    @Override
    public void execute() {

        drive.robotDrive(-1, 0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Command is never finished.
    @Override
    public boolean isFinished() {
        return isAbsPitchDecreasing();
    }

};
