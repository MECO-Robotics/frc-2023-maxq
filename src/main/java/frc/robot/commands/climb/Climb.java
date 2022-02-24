// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;

import java.util.concurrent.DelayQueue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Climb the middle and traversal rungs. Assumes the robot is already positioned
 * at the
 * middle rung and the upper winch is ready to be pulled.
 * 
 */
public class Climb extends SequentialCommandGroup {

  /**
   * Creates a new Command.
   *
   * @param ballCollectionSubsystem The subsystem used by this command.
   */
  public Climb(ClimbingSubsystem climbingSubsystem) {

    addCommands(
        new RotatingArmLowerToPosition(climbingSubsystem, .9),    //Grip the middle bar
        new TelescopingArmSet(climbingSubsystem, 1),    //Lets the telescoping arm all the way
        new RotatingArmLowerToPosition(climbingSubsystem, 0),   //Pulls the bot upwards
        new TelescopingArmSet(climbingSubsystem, .95),    //Hooks onto the traversal bar
        new RotatingArmLowerToPosition(climbingSubsystem, .5)   //Releases the grip on middle bar

    );
  }

}
