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
import frc.robot.commands.AutoShootCollectRightShoot;
import frc.robot.commands.MoveOctagon;
import frc.robot.commands.PlusSign;
import frc.robot.commands.Shift;
import frc.robot.commands.Stop;
import frc.robot.commands.TeleopDrive;
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
  // private final CargoSubsystem ballCollectionSubsystem = new CargoSubsystem();
  // private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
  private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem();

  // private final TeleopBallCollection teleopBallCollection = new TeleopBallCollection(ballCollectionSubsystem, controllerSubsystem);
  
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
    autoCommands.put("PlusSign", new PlusSign(driveSubsystem));
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

    JoystickButton shiftDown = new JoystickButton(
      controllerSubsystem.getPilotController(), 
      XboxController.Button.kLeftBumper.value);

    shiftDown.whenPressed(new Shift(driveSubsystem, false), false);

    JoystickButton shiftUp = new JoystickButton(
      controllerSubsystem.getPilotController(), 
      XboxController.Button.kRightBumper.value);

    shiftUp.whenPressed(new Shift(driveSubsystem, true), false);
    

    /*
    JoystickButton extend = new JoystickButton(controllerSubsystem.getPilotController(), XboxController.Button.kY.value);
    extend.whenHeld(new ExtendPneumatic(climbingSubsystem));

    JoystickButton contract = new JoystickButton(controllerSubsystem.getPilotController(), XboxController.Button.kX.value);
    contract.whenHeld(new ContractPneumatic(climbingSubsystem));
*/
/*
    JoystickButton forward = new JoystickButton(controllerSubsystem.getPilotController(), XboxController.Button.kY.value);
    forward.whenHeld(new PneumaticCommand(climbingSubsystem, Value.kForward));

    JoystickButton reverse = new JoystickButton(controllerSubsystem.getPilotController(), XboxController.Button.kX.value);
    reverse.whenHeld(new PneumaticCommand(climbingSubsystem, Value.kReverse));
*/
    
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

  // public Command getTeleopBallCollection() {
  //   return teleopBallCollection;
  // }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}
