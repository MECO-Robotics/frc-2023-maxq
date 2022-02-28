// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 *  The code will allow you to set the telescoping arm.
 */
public class TelescopingArmOut extends CommandBase {

  private final ClimbingSubsystem climb;
  
  /**
   * Creates a new ExampleCommand. Manual controls.
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   * @param length How far out to set it. 0= all the way in, 1=all the way out, .5=1/2 way out, etc...
   */
  public TelescopingArmOut(ClimbingSubsystem subsystem) {
    climb = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.telescopingArmWinchMove(.5, .5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.telescopingArmWinchMove(0, 0);
  }

  /**
   * Finished when the distance between the desired length and the actual length
   * according to the subsystem is less than 1% difference.
   */
  @Override
  public boolean isFinished() {
    return climb.getTelescopingArmLeftWinchPosition() > .95 && climb.getTelescopingArmRightWinchPosition() > .95;
  }
}
