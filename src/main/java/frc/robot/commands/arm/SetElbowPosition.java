package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;

public class SetElbowPosition extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ElbowPosition elbowPositionVar;
    

    
    public SetElbowPosition(ArmSubsystem arm, Constants.ElbowPosition elbowPosition) {
        armSubsystem = arm;
        elbowPositionVar = elbowPosition;
        // Only add the arm. Don't want usage of the controller to be exclusive
        // NOTE: Remove this if we want to support concurrent manual control while also
        // providing other commands
        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(elbowPositionVar);
        

       
    }

    @Override
    public boolean isFinished() {
        return false; // this command never finishes (but can be interupted)
    }
}
