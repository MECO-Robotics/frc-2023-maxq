package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;

public class GripperManualControl extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ControllerSubsystem controllerSubsystem;

    public GripperManualControl(ArmSubsystem armIn, ControllerSubsystem controlIn) {
        armSubsystem = armIn;
        controllerSubsystem = controlIn;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.moveGripper(controllerSubsystem.getCopilotController().getRightTriggerAxis()
                - controllerSubsystem.getCopilotController().getLeftTriggerAxis());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
