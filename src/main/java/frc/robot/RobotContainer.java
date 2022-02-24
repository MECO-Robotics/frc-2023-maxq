// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AutoShootCollectRightShoot;
import frc.robot.commands.auto.BallAuto;
import frc.robot.commands.cargo.Intake;
import frc.robot.commands.cargo.Outtake;
import frc.robot.commands.demo.MoveOctagon;
import frc.robot.commands.drive.Shift;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CargoSubsystem cargoSubsystem = new CargoSubsystem();
  private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
  private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
  
  private final Map<String, Command> autoCommands = new HashMap<>();
  private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

  private final SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();

  public enum DriveMode {
    SplitArcade,
    Tank,
    Joystick
  }

  /** The container for the robot. Contains subsystems, I/O devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    

    // ------------------------------------------------------------------------
    //
    //                 PLACE ALL AUTONOMOUS COMMANDS HERE
    //

    // Create a mapping of name to command object for every autonomous command
    // By using the class name as the name, it will be easy to remember which goes with which.
    autoCommands.put("MoveOctagon", new MoveOctagon(driveSubsystem));
    autoCommands.put("BallAuto", new BallAuto(driveSubsystem, cargoSubsystem));
    autoCommands.put("AutoShootCollectRightShoot", new AutoShootCollectRightShoot(driveSubsystem));


    // 
    //
    // ------------------------------------------------------------------------

    for(String choiceName : autoCommands.keySet()) {
      autoCommandChoice.addOption(choiceName, choiceName);
    }
    autoCommandChoice.setDefaultOption("AutoShootCollectRightShoot", "AutoShootCollectRightShoot");
    SmartDashboard.putData("Autonomous Mode", autoCommandChoice);
    
    driveMode.setDefaultOption("Split Arcade", DriveMode.SplitArcade);
    driveMode.addOption("Tank", DriveMode.Tank);
    driveMode.addOption("Joystick", DriveMode.Joystick);
    SmartDashboard.putData("Drive mode", driveMode);

    // SmartDashboard.putData("ARM UP", new RaiseCargoWrist(ballCollectionSubsystem));
    // SmartDashboard.putData("ARM DOWN", new LowerCargoWrist(ballCollectionSubsystem));
    SmartDashboard.putData("SHIFT DOWN", new Shift(driveSubsystem, false));
    SmartDashboard.putData("SHIFT UP", new Shift(driveSubsystem, true));
    // SmartDashboard.putData("INTAKE", new Intake(ballCollectionSubsystem));
    // SmartDashboard.putData("OUTTAKE", new Outtake(ballCollectionSubsystem));
    // Set default commands
    driveSubsystem.setDefaultCommand(new Stop(driveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // This is an example of button bindings using the new approach. The rest
    // of the button bindings are actually done in the ControllerSubsystem class.

    configureTeleopDriveButtonBindings();
    
    configureTeleopCargoButtonBindings();

    configureTeleopClimbButtonBindings();
  }

  /**
   * Setup the buttons for controlling climbing during teleop
   */
  private void configureTeleopClimbButtonBindings() {
  }

  /**
   * Setup the buttons for collecting and shooting cargo
   */
  private void configureTeleopCargoButtonBindings() {
    XboxController copilot = controllerSubsystem.getCopilotController();
    POVButton upPovButton = new POVButton(copilot, 0);
    POVButton righttPovButton = new POVButton(copilot, 90);
    POVButton downnPovButton = new POVButton(copilot, 180);
    POVButton leftPovButton = new POVButton(copilot, 270);

    // Uncomment when cargo subsystem is available
    //upPovButton.whenHeld(new Intake(cargoSubsystem));
    // downnPovButton.whenHeld(new Outtake(cargoSubsystem));
  }

  /**
   * Setup the buttons for teleop drive.
   */
  private void configureTeleopDriveButtonBindings() {

    JoystickButton leftBumper = new JoystickButton(
      controllerSubsystem.getPilotController(), 
      XboxController.Button.kLeftBumper.value);

    leftBumper.whenPressed(new Shift(driveSubsystem, false), false);

    JoystickButton rightBumper = new JoystickButton(
      controllerSubsystem.getPilotController(), 
      XboxController.Button.kRightBumper.value);

    rightBumper.whenPressed(new Shift(driveSubsystem, true), false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = null;

    if(autoCommandChoice != null && autoCommandChoice.getSelected() != null) {
      command = autoCommands.get(autoCommandChoice.getSelected());
    }

    return command;
  }

  public Command getTeleopDriveCommand() {
    return new TeleopDrive(driveSubsystem, controllerSubsystem, driveMode.getSelected());
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}
