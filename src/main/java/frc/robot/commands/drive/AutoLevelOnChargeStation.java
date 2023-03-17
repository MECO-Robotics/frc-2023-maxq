// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command to auto level on the charge station. See the
 * DriveSubsystem.chargeStationEnergize() method for requirements.
 * Command is finished when the robot is level and stationary
 */
public class AutoLevelOnChargeStation extends CommandBase {

    private final DriveSubsystem driveTrain;
    private boolean energized = false;

    public AutoLevelOnChargeStation(DriveSubsystem driveSubsystem) {

        driveTrain = driveSubsystem;

        // DO NOT REQUIRE the drive subsystem as a required subsystem. We can use this during teleop
        // while performing other actions.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        energized = driveTrain.chargeStationEnergize();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

    // Command is never finished.
    @Override
    public boolean isFinished() {
        return false;
    }

};
