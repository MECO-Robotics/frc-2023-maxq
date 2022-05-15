package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LightSubsystem;

public class Strobe extends CommandBase {
    private final LightSubsystem lights;

    public Strobe(LightSubsystem lightSubsystem) {
        lights = lightSubsystem;

    }

    
    @Override
    public void execute() {
        lights.strobe();
        
    }




    @Override
    public boolean isFinished() {
        return false;
    }

}
