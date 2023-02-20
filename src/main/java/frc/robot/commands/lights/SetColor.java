package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class SetColor extends CommandBase {
    
    private final LightSubsystem lights;
    private final Color color;

    public SetColor(LightSubsystem lightSubsystem, Color colour) {
        lights = lightSubsystem;
        color = colour;
    }

    @Override
    public void execute() {
        lights.set(color);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
