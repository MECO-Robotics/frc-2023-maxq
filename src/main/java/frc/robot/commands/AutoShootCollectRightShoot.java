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
public class AutoShootCollectRightShoot extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public AutoShootCollectRightShoot(DriveSubsystem driveSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands scheduled will do that.


    // For reference, when using Spin(Right/Left)Distance, 81 inches is 360 degrees on the simulator. Need to see on Grommet what it is.
    addCommands(

        //shoot
        new DriveBackward(driveSubsystem, 5.5*12),      // ~6'
        new SpinRightDistance(driveSubsystem, 28.0),  // 150 degrees
        new DriveForward(driveSubsystem, 3*12),       // ~4'
        //intake
        new DriveBackward(driveSubsystem, 3*12),      // ~4'
        new SpinLeftDistance(driveSubsystem, 30.0),   // 150 degrees
        new DriveForward(driveSubsystem, 5.5*12)        // ~6'
        //shoot

    );
  }
}
