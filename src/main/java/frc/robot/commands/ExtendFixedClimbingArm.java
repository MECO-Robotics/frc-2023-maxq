// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 *       The code will allow you to raise and lower the fixed arm with the winch.
 * 
 */
public class ExtendFixedClimbingArm extends CommandBase {

  private final ClimbingSubsystem climbingSubsystem;
  
//variable name always starts with a lowercase letter
  /**
   * Creates a new ExampleCommand. Manual controls.
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public ExtendFixedClimbingArm(ClimbingSubsystem subsystem) {
    climbingSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Trigger moving up
    //get current encoder position of the lower arm/fixed arm winch
    climbingSubsystem.unwindLowerArmWinch();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Just keep checking the arm movement, without the up button pressed
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    // If the motor speed is zero, we're done. This is checked after
    // the initialize() and execute() methods, which will set the arm
    // speed to non-zero.
    return climbingSubsystem.isTheLowerWinchFinishedWinding();
  }
}
