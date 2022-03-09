// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.cargo.LowerCargoElbow;
import frc.robot.commands.cargo.LowerCargoWrist;
import frc.robot.commands.cargo.Outtake;
import frc.robot.commands.cargo.RaiseCargoElbow;
import frc.robot.commands.cargo.RaiseCargoWrist;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CargoSubsystem;

/** 
 * Drives the robot autonomously in a prescribed pattern. 
 *
 * Refer to this diagram for the routine:
 * https://docs.google.com/drawings/d/1GxrTwsLhETpqVrT3ycNIkLCBsJlLdFQTz0Ca2pnxkw0/edit 
 */
public class SpinShoot extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public SpinShoot(DriveSubsystem driveSubsystem, CargoSubsystem cargoSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands scheduled will do that.
    
    addCommands(
      
    //face the hub
    new DriveStraight(driveSubsystem, -2*12),
    new LowerCargoWrist(cargoSubsystem),
    new RaiseCargoElbow(cargoSubsystem),
    new DriveStraight(driveSubsystem, 2*12),
    new Outtake(cargoSubsystem).withTimeout(5),
   new DriveStraight(driveSubsystem, -7*12),
    new LowerCargoElbow(cargoSubsystem),
   new RaiseCargoWrist(cargoSubsystem)


    
    );
  }
}
