// Copyright (c) MECO Robotics
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
import frc.robot.commands.drive.AutoLevelOnChargeStation;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PowerHub;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
    private final LightSubsystem lightSubsystem = new LightSubsystem();
    // private final PowerHub powerHub = new PowerHub();

    private final Map<String, Command> autoCommands = new HashMap<>();
    private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

    private final SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();

    public enum DriveMode {
        RobotOrientedHolonomic,
        FieldOrientedHolonomic,
        SplitArcade,
        Tank
    }

    /**
     * The container for the robot. Contains subsystems, I/O devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // ------------------------------------------------------------------------
        //
        // PLACE ALL AUTONOMOUS COMMANDS HERE
        //

        // Create a mapping of name to command object for every autonomous command
        // By using the class name as the name, it will be easy to remember which goes
        // with which.

        //
        //
        // ------------------------------------------------------------------------

        for (String choiceName : autoCommands.keySet()) {
            autoCommandChoice.addOption(choiceName, choiceName);
        }
        autoCommandChoice.setDefaultOption("DriveBack", "DriveBack");
        SmartDashboard.putData("Autonomous Mode", autoCommandChoice);

        driveMode.setDefaultOption("Robot Oriented", DriveMode.RobotOrientedHolonomic);
        driveMode.addOption("Field Oriented", DriveMode.FieldOrientedHolonomic);
        driveMode.addOption("Split Arcade", DriveMode.SplitArcade);
        driveMode.addOption("Tank", DriveMode.Tank);
        SmartDashboard.putData("Drive mode", driveMode);

        // Set default commands
        driveSubsystem.setDefaultCommand(new Stop(driveSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // This is an example of button bindings using the new approach. The rest
        // of the button bindings are actually done in the ControllerSubsystem class.

        // configureTeleopDriveButtonBindings();
    }

    /**
     * Setup the buttons for controlling climbing during teleop
     */
    private void configureTeleopClimbButtonBindings() {

        // NATE: bind buttons for teleop climb
        XboxController copilot = controllerSubsystem.getCopilotController();
        JoystickButton leftBumper = new JoystickButton(copilot, XboxController.Button.kLeftBumper.value);
        JoystickButton righBumper = new JoystickButton(copilot, XboxController.Button.kRightBumper.value);
        JoystickButton xButton = new JoystickButton(copilot, XboxController.Button.kX.value);
        JoystickButton yButton = new JoystickButton(copilot, XboxController.Button.kY.value);
        JoystickButton bButton = new JoystickButton(copilot, XboxController.Button.kB.value);

        XboxController pilot = controllerSubsystem.getPilotController();
        JoystickButton xButtonpilot = new JoystickButton(pilot, XboxController.Button.kX.value);
        JoystickButton yButtonpilot = new JoystickButton(pilot, XboxController.Button.kY.value);

        // leftBumper.whenPressed(new TeleopRotatingArmPneumaticIn(climbingSubsystem),
        // false);
        // righBumper.whenPressed(new TeleopRotatingArmPneumaticOut(climbingSubsystem),
        // false);
        // xButton.whenPressed(new TeleopRotatingArmPneumaticOff(climbingSubsystem),
        // false);
        // bButton.whenPressed(new Climb(climbingSubsystem), false);
        // yButton.whenPressed(new TeleopRotatingArmPneumaticOut(climbingSubsystem),
        // false);
    }

    /**
     * Setup the buttons for teleop drive.
     */
    private void configureTeleopDriveButtonBindings() {

        XboxController pilot = controllerSubsystem.getPilotController();
        JoystickButton aButton = new JoystickButton(pilot, XboxController.Button.kA.value);

        // Whenever holding A - run the auto level routine. When not holding, do normal
        // driving
        aButton.whileTrue(new AutoLevelOnChargeStation(driveSubsystem))
                .whileFalse(new TeleopDrive(driveSubsystem, controllerSubsystem, DriveMode.RobotOrientedHolonomic));

        // TODO: Bind buttons for the ResetSensors command. Brent recommands a "two man
        // rule" for engaging, requiring a button on the pilot and co-pilot to press a
        // button at the same time in order to reset
    }

    int testWheel = Constants.FRONT_LEFT_CAN;

    public void testMode() {
        XboxController pilot = controllerSubsystem.getPilotController();
        double testThrottle = pilot.getLeftY();

        if (pilot.getYButtonReleased()) {
            testWheel = Constants.FRONT_LEFT_CAN;
        } else if (pilot.getXButtonReleased()) {
            testWheel = Constants.FRONT_RIGHT_CAN;
        } else if (pilot.getBButtonReleased()) {
            testWheel = Constants.BACK_LEFT_CAN;
        } else if (pilot.getAButtonReleased()) {
            testWheel = Constants.BACK_RIGHT_CAN;
        }

        driveSubsystem.runWheel(testWheel, testThrottle);
    };

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command command = null;

        if (autoCommandChoice != null && autoCommandChoice.getSelected() != null) {
            command = autoCommands.get(autoCommandChoice.getSelected());
        }

        return command;
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public Command getTeleopCommand() {
        return new TeleopDrive(driveSubsystem, controllerSubsystem, driveMode.getSelected());
    }
}
