// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.ArmLoadingStation;
import frc.robot.commands.arm.ArmStow;
import frc.robot.commands.arm.GoNodeHigh;
import frc.robot.commands.arm.GoNodeMid;
import frc.robot.commands.arm.TeleopArmControl;
import frc.robot.commands.autonomous.AutoTest;
import frc.robot.commands.brakes.LowerBrakes;
import frc.robot.commands.brakes.RaiseBrakes;
import frc.robot.commands.drive.AutoLevelOnChargeStation;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.lights.Blink;
import frc.robot.commands.lights.SetColor;
import frc.robot.commands.lights.SetColorToAlliance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BrakesSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
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
    private final BrakesSubsystem brakesSubsystem = new BrakesSubsystem();

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
        SmartDashboard.putData(new AutoTest(driveSubsystem));
        SmartDashboard.putData("red", new SetColor(lightSubsystem, Color.kRed));
        SmartDashboard.putData("green",new SetColor(lightSubsystem, Color.kGreen));
        SmartDashboard.putData("blue",new SetColor(lightSubsystem, Color.kBlue));

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
    public void configureButtonBindings() {

        XboxController pilot = controllerSubsystem.getPilotController();
        JoystickButton pilotAButton = new JoystickButton(pilot, XboxController.Button.kA.value);
        JoystickButton pilotBButton = new JoystickButton(pilot, XboxController.Button.kB.value);
        JoystickButton pilotXButton = new JoystickButton(pilot, XboxController.Button.kX.value);
        JoystickButton pilotYButton = new JoystickButton(pilot, XboxController.Button.kY.value);
        JoystickButton pilotStartButton = new JoystickButton(pilot, XboxController.Button.kStart.value);

        XboxController coPilot = controllerSubsystem.getCopilotController();
        JoystickButton coPilotAButton = new JoystickButton(coPilot, XboxController.Button.kA.value);
        JoystickButton coPilotBButton = new JoystickButton(coPilot, XboxController.Button.kB.value);
        JoystickButton coPilotXButton = new JoystickButton(coPilot, XboxController.Button.kX.value);
        JoystickButton coPilotYButton = new JoystickButton(coPilot, XboxController.Button.kY.value);
        JoystickButton coPilotRightBumper = new JoystickButton(coPilot, XboxController.Button.kRightBumper.value);
        JoystickButton coPilotStartButton = new JoystickButton(coPilot, XboxController.Button.kStart.value);

        JoystickButton triggerJoystickButton = new JoystickButton(pilot, XboxController.Button.kY.value);
        POVButton dpadButton = new POVButton(pilot, 0);

        pilotBButton.toggleOnTrue(new LowerBrakes(brakesSubsystem));
        pilotBButton.toggleOnFalse(new RaiseBrakes(brakesSubsystem));

        // Whenever holding X - run autolevel command.
        pilotXButton.whileTrue(new AutoLevelOnChargeStation(driveSubsystem));

        // pilot status light controls
        pilotYButton.toggleOnTrue(new Blink(lightSubsystem, Color.kYellow));
        pilotYButton.toggleOnFalse(new SetColorToAlliance(lightSubsystem));
        pilotAButton.toggleOnTrue(new Blink(lightSubsystem, Color.kPurple));
        pilotAButton.toggleOnFalse(new SetColorToAlliance(lightSubsystem));

        // :)

        // coPilot controls
        coPilotAButton.onTrue(new ArmIntake(armSubsystem));
        coPilotRightBumper.onTrue(new ArmStow(armSubsystem));
        coPilotBButton.onTrue(new GoNodeMid(armSubsystem));
        coPilotYButton.onTrue(new GoNodeHigh(armSubsystem));
        coPilotXButton.onTrue(new ArmLoadingStation(armSubsystem));

        // TODO: Redo commands
        // 1. Change all arm commands to work like the following:
        // coPilotAButton.onTrue(makeArmCommand(new ArmIntake(armSubsystem)));
        //
        // 2. Change all commands to add the addRequirements(armSubsystem); to their
        // constructors.
        //
        // 3. Change the TeleopArmControl to use the addRequirements(armSubsystem); in
        // it's constructor


        // reset sensors - copilot and pilot
        pilotStartButton.and(coPilotStartButton).onTrue(new ResetSensors(driveSubsystem));
    }


    Command makeArmCommand(Command armCmd) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                    armCmd, 
                    new WaitCommand(5.0)),
                new TeleopArmControl(armSubsystem, controllerSubsystem));
    }
    // --------------------------------------------------------------------

    public void testInit() {

    }

    private int testWheel = Constants.FRONT_LEFT_CAN;

    public void testPeriodic() {
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

        double shoulder = controllerSubsystem.getCopilotController().getLeftX();
        double elbow = controllerSubsystem.getCopilotController().getRightX();
        double gripper = (controllerSubsystem.getCopilotController().getRightTriggerAxis()
                - controllerSubsystem.getCopilotController().getLeftTriggerAxis());

        if (logger++ % 50 == 0)
            System.out.println(String.format("TEST Gripper: R:%f, L:%f G:%f",
                    controllerSubsystem.getCopilotController().getRightTriggerAxis(),
                    controllerSubsystem.getCopilotController().getLeftTriggerAxis(),
                    gripper));
        armSubsystem.manualControlNoLimits(elbow, shoulder, gripper);
    };

    static int logger = 0;

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