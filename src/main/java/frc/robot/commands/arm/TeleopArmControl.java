package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;

public class TeleopArmControl extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ControllerSubsystem controllerSubsystem;

    public TeleopArmControl(ArmSubsystem arm, ControllerSubsystem controller) {
        armSubsystem = arm;
        controllerSubsystem = controller;

        // Only add the arm. Don't want usage of the controller to be exclusive
        // NOTE: Remove this if we want to support concurrent manual control while also
        // providing other commands
        addRequirements(arm);
    }

    @Override
    public void execute() {

        double shoulder = controllerSubsystem.getCopilotController().getLeftX();
        double elbow = controllerSubsystem.getCopilotController().getRightX();
        double gripper = controllerSubsystem.getCopilotController().getRightY();

        armSubsystem.manualControl(elbow, shoulder, gripper);
    }

    @Override
    public boolean isFinished() {
        return false; // this command never finishes (but can be interupted)
    }
}
