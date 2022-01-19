// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** 
 * Drives the robot autonomously in a prescribed pattern. 
 *
 * Refer to this diagram for the routine:
 * https://docs.google.com/drawings/d/1GxrTwsLhETpqVrT3ycNIkLCBsJlLdFQTz0Ca2pnxkw0/edit 
 */
public class AutoShootcollect extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public AutoShootcollect(DriveSubsystem driveSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands
    // scheduled will do that.

    addCommands(
        //shoot
        new DriveBackward(driveSubsystem, 20),
        new SpinRightDistance(driveSubsystem, 20),  // 150
        new DriveForward(driveSubsystem, 20),
        //intake
        new DriveBackward(driveSubsystem, 20),
        new SpinRightDistance(driveSubsystem, -20),
        new DriveForward(driveSubsystem, 20)
        //shoot

    );

  }
}
