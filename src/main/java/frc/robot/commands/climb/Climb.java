// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    // We don't want to call addRequirements. If we did, we're saying this is a
    // command that causes other commands currently running on the 
    // ClimbingSubystem to be inerrupted. 
    // Since this command is invoked by the copilot, we want him to still
    // be able to manually control



    // Initial conditions:
    // 1) The rotating arm is fully raised (1.0 position)
    // 2) The telescoping arm is fully contracted (0.0 position)
    // 3) The rotating arm gripper is touching the middle rung
    // 4) The robot rear is facing the back of the field wall:

    /*
     * 
     * T
     * H
     * M >
     * ?\ L
     * | \
     * =0=0=0=
     */

    // Final conditions:
    // 1) The robot is hanging only by the telescoping arm

    addCommands(
       
        new RotatingArmGrabBar(climbingSubsystem), // Grip the middle bar (pull the winch in a bit)
        new TelescopingArmOut(climbingSubsystem), // 
        new RotatingArmLowerFull(climbingSubsystem), // bring rotating arm all the way down
        new TelescopingArmGrabBar(climbingSubsystem), // grab the traversal bar by pulling the telescoping arm in a little
        new TelescopingArmIn(climbingSubsystem),      // bring the telescoping bar in all the way
        new RotatingArmRaiseFullOpenGrip(climbingSubsystem) // Releases the grip on middle bar (release tension on the winch until it lets go)

    );

    // We want to be able to interrupt this routine and take over using manual controls,
    // by adding in the requirements, and then using the "Cancel Climb" command, that command
    // will also interrupt this routine and reschedule teleop.
    addRequirements(climbingSubsystem);
  }

}
