package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;

public class SetArmPosition extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ControllerSubsystem controllerSubsystem;

    public SetArmPosition(ArmSubsystem arm, ControllerSubsystem controller) {
        armSubsystem = arm;
        controllerSubsystem = controller;

        // Only add the arm. Don't want usage of the controller to be exclusive
        // NOTE: Remove this if we want to support concurrent manual control while also
        // providing other commands
        addRequirements(arm);
    }

    @Override
    public void execute() {

        boolean copilotConnected = controllerSubsystem.getCopilotController() != null;

        // System.out.println(String.format("Brennan's awesome print statement 2: COPILOT Connected: %s",
        //         copilotConnected ? "YES" : "NO"));

        // Only run the arm if we have a copilot controller connected.
        if (copilotConnected) {

            double shoulder = controllerSubsystem.getCopilotController().getLeftX();
            double elbow = controllerSubsystem.getCopilotController().getRightX();
            double gripper = controllerSubsystem.getCopilotController().getRightY();

            //armSubsystem.armPositionControl(null, null, null);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // this command never finishes (but can be interupted)
    }
}
