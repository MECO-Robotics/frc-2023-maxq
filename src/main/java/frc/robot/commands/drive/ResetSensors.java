// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetSensors extends CommandBase {
    private final DriveSubsystem driveTrain;

    /**
     * Creates a new Ecommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public ResetSensors(DriveSubsystem driveSubsystem) {
        driveTrain = driveSubsystem;

        // Do NOT make the driveSubsystem a required system. The command is instananeous and
        // would still allow normal teleop drive at the same time.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveTrain.resetSensors();
    }

    // Returns true when the command should end. (this command never finishes)
    @Override
    public boolean isFinished() {
        return true;
    }
}
