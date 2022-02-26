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

    // Initial conditions:
    //  1) The rotating arm is fully raised (1.0 position)
    //  2) The telescoping arm is fully contracted (0.0 position)
    //  3) The rotating arm gripper is touching the middle rung
    //  4) The robot rear is facing the back of the field wall:

    /*

        T
              H
                    M >
                      ?\   L
                      | \
                   =0=0=0=
    */

    // Final conditions:
    //  1) The robot is hanging only by the telescoping arm    

    addCommands(
        new RotatingArmLowerToPosition(climbingSubsystem, 1),     // Grip the middle bar (pull the winch in a bit)
        new TelescopingArmSet(climbingSubsystem, .95),            // will lock the rotating arm onto the bar
        new RotatingArmLowerToPosition(climbingSubsystem, -.95),  // Lets the telescoping arm all the way
        new TelescopingArmSet(climbingSubsystem, 1),              // Fully exends the telescpoing arm
        new RotatingArmLowerToPosition(climbingSubsystem, 0),     // Pulls the bot upwards (pull the winch in all the way)
        new TelescopingArmSet(climbingSubsystem, .95),            // Hooks onto the traversal bar (drop the telescoping arm a little)
        new RotatingArmLowerToPosition(climbingSubsystem, .5)     // Releases the grip on middle bar (release tension on the winch until it lets go)

    );
  }

}
