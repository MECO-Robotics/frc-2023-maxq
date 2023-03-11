// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;
import frc.robot.Constants.GripperPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;

public class SetGripperPosition extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final GripperPosition GripperPositionVar;
    

    
    public SetGripperPosition(ArmSubsystem arm, Constants.GripperPosition GripperPosition) {
        armSubsystem = arm;
        GripperPositionVar = GripperPosition;
        // Only add the arm. Don't want usage of the controller to be exclusive
        // NOTE: Remove this if we want to support concurrent manual control while also
        // providing other commands
        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(GripperPositionVar);
        

       
    }

    @Override
    public boolean isFinished() {
        return false; // this command never finishes (but can be interupted)
    }
}
