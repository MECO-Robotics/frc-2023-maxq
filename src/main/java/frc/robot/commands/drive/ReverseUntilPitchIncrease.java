// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;

public class ReverseUntilPitchIncrease extends PitchChangeCommandBase {

    private final DriveSubsystem drive;

    public ReverseUntilPitchIncrease(DriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        drive = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        super.addPitchSample();
        drive.robotDrive(-1, 0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Command is never finished.
    @Override
    public boolean isFinished() {
        return super.isAbsPitchIncreasing();
    }

};
