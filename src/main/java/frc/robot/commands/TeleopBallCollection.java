// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BallCollectionSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopBallCollection extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final BallCollectionSubsystem ballCollector;
  private final ControllerSubsystem controllers;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ballCollectionSubsystem The subsystem used by this command.
   */
  public TeleopBallCollection(BallCollectionSubsystem ballCollectionSubsystem, ControllerSubsystem controllerSubsystem) {
    ballCollector = ballCollectionSubsystem;
    controllers = controllerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballCollectionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ballCollector.moveArm(
      controllers.getArmUpButton(), 
      controllers.getArmDownButton());
    
    ballCollector.moveIntake(
      controllers.getIntakeThrottle(), 
      controllers.getOuttakeThrottle());

    ballCollector.manualControl(
      controllers.getManualArm(), 
      controllers.getManualIntake());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }
}
