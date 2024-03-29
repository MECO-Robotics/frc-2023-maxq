// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
        addRequirements(arm);
    }

    int logger = 0;
    @Override
    public void execute() {

        //if(logger++ % 10 ==0 ) System.out.println(String.format("TeleoparmControl"));
        boolean copilotConnected = controllerSubsystem.getCopilotController() != null;

        // Only run the arm if we have a copilot controller connected.
        if (copilotConnected) {

            double shoulder = -controllerSubsystem.getCopilotController().getLeftY();
            double elbow = -controllerSubsystem.getCopilotController().getRightY();
            double gripper = (controllerSubsystem.getCopilotController().getRightTriggerAxis()
             - controllerSubsystem.getCopilotController().getLeftTriggerAxis());
            
            armSubsystem.manualControl(elbow, shoulder);
            armSubsystem.moveGripper(gripper);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // this command never finishes (but can be interrupted)
    }
}
