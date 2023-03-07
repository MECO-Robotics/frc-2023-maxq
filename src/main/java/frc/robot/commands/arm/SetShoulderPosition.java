package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;
import frc.robot.Constants.ShoulderPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;

public class SetShoulderPosition extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ShoulderPosition ShoulderPositionVar;
    

    
    public SetShoulderPosition(ArmSubsystem arm, Constants.ShoulderPosition ShoulderPosition) {
        armSubsystem = arm;
        ShoulderPositionVar = ShoulderPosition;
        // Only add the arm. Don't want usage of the controller to be exclusive
        // NOTE: Remove this if we want to support concurrent manual control while also
        // providing other commands
        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(ShoulderPositionVar);
        

       
    }

    @Override
    public boolean isFinished() {
        return false; // this command never finishes (but can be interupted)
    }
}
