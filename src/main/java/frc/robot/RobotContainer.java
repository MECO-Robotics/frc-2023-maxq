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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm.TeleopArmControl;
import frc.robot.commands.drive.AutoLevelOnChargeStation;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.lights.SetColor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
    private final LightSubsystem lightSubsystem = new LightSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    // private final PowerHub powerHub = new PowerHub();

    private final Map<String, Command> autoCommands = new HashMap<>();
    private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

    private final SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();

    public enum DriveMode {
        RobotOrientedHolonomic,
        RobotOrientedOpenLoop,
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

        driveMode.setDefaultOption("Robot Open Loop", DriveMode.RobotOrientedOpenLoop);
        driveMode.addOption("Robot Closed Loop", DriveMode.RobotOrientedHolonomic);
        driveMode.addOption("Field", DriveMode.FieldOrientedHolonomic);
        driveMode.addOption("Split Arcade", DriveMode.SplitArcade);
        driveMode.addOption("Tank", DriveMode.Tank);
        SmartDashboard.putData("Drive mode", driveMode);

        // Set default commands
        driveSubsystem.setDefaultCommand(new Stop(driveSubsystem));

        // In WPILib 2023, subsystems are no longer automatically added to the smart
        // dashboard. you have to add them yourself.
        // however, in test mode, they are added so it might appear twice in test mode.
        SmartDashboard.putData("Mecanum Drive", driveSubsystem);
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

        configureTeleopDriveButtonBindings();
    }

    /**
     * Setup the buttons for teleop drive.
     */
    private void configureTeleopDriveButtonBindings() {

        XboxController pilot = controllerSubsystem.getPilotController();
        JoystickButton aButton = new JoystickButton(pilot, XboxController.Button.kA.value);
        JoystickButton bButton = new JoystickButton(pilot, XboxController.Button.kB.value);
        JoystickButton xButton = new JoystickButton(pilot, XboxController.Button.kX.value);
        JoystickButton yButton = new JoystickButton(pilot, XboxController.Button.kY.value);
        JoystickButton triggerJoystickButton = new JoystickButton(pilot, XboxController.Button.kY.value);
        POVButton dpadButton = new POVButton(pilot, 0);

        // Whenever holding X - run autolevel command.
        xButton.whileTrue(new AutoLevelOnChargeStation(driveSubsystem));

        yButton.onTrue(new SetColor(lightSubsystem, Color.kYellow));

        aButton.onTrue(new SetColor(lightSubsystem, Color.kPurple));

        // TODO: Brent recommands a "two man
        // rule" for engaging, requiring a button on the pilot and co-pilot to press a
        // button at the same time in order to reset
        // aButton.whileTrue(new ResetSensors(driveSubsystem));
    }

    // --------------------------------------------------------------------

    private int testWheel = Constants.FRONT_LEFT_CAN;

    public void testMode() {
        XboxController pilot = controllerSubsystem.getPilotController();
        double testThrottle = pilot.getLeftY() * .25;

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

    // --------------------------------------------------------------------

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
        Command command;

        // Arm and Drive
        command = new ParallelCommandGroup(
                new TeleopDrive(driveSubsystem, controllerSubsystem, driveMode.getSelected()),
                new TeleopArmControl(armSubsystem, controllerSubsystem));

        // Drive only
        // command = new TeleopDrive(driveSubsystem, controllerSubsystem,
        // driveMode.getSelected());

        return command;
    }

    public void robotPeriodic() {

        if (visionSubsystem.hasValidPose()) {
            driveSubsystem.addVisionMeasurement(
                    visionSubsystem.getVisionMeasurement(),
                    visionSubsystem.getVisionMeasurementTimestamp());
        }
    }
}