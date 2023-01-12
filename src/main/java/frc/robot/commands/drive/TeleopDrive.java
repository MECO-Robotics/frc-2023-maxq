// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.RobotContainer;
import frc.robot.RobotContainer.DriveMode;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final DriveSubsystem driveTrain;
  private final ControllerSubsystem controllers;
  private final DriveMode driveMode;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public TeleopDrive(DriveSubsystem driveSubsystem, ControllerSubsystem controllerSubsystem, RobotContainer.DriveMode driveMode) {
    driveTrain = driveSubsystem;
    controllers = controllerSubsystem;
    this.driveMode = driveMode;
    
    // Cancel any other currently running drive subsystem commands before we run.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(driveMode) {
      case SplitArcade:
       // driveTrain.arcadeDrive(controllers.getThrottle(), controllers.getTurn());
        break;
      
      case Tank: 
       // driveTrain.tankDrive(controllers.getTankLeft(), controllers.getTankRight());
        break;
      
      case Joystick:
       // driveTrain.arcadeDrive(controllers.getJoystickY(), controllers.getJoystickX());
        break;
    }
    
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
