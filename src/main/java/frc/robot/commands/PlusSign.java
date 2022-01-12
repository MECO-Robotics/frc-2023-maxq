// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Drives the robot autonomously in a plus sign pattern. */
public class PlusSign extends SequentialCommandGroup {
  
  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public PlusSign(DriveSubsystem driveSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the commands
    // scheduled will do that.

    for(int i = 0; i < 4; i++){  
      addCommands(
        new DriveForward(driveSubsystem, 20),
        new DriveBackward(driveSubsystem, 20),
        new SpinRightAngle(driveSubsystem, 90)
      );
    }
  }
}
