// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.brakes.LowerBrakes;
import frc.robot.commands.brakes.RaiseBrakes;
import frc.robot.commands.drive.ReverseUntilPitchDecrease;
import frc.robot.commands.drive.ReverseUntilPitchIncrease;
import frc.robot.commands.drive.Stop;
import frc.robot.subsystems.BrakesSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Auto level on charge station
 */
public class AutoLevel extends SequentialCommandGroup {

    public AutoLevel(DriveSubsystem drive, BrakesSubsystem brakes) {

        addCommands(
                new RaiseBrakes(brakes),
                new ReverseUntilPitchIncrease(drive),
                new ReverseUntilPitchDecrease(drive),
                new Stop(drive),
                new LowerBrakes(brakes));

    }
}
