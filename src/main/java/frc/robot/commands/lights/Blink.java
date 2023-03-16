// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class Blink extends CommandBase {
    private final LightSubsystem lights;
    int i = 0;
    Color desiredColor = Color.kBlack;
    Color currentColor = Color.kBlack;

    public Blink(LightSubsystem lightSubsystem, Color color) {
        lights = lightSubsystem;
        desiredColor = color;

        addRequirements(lightSubsystem);
    }
    
    //This line of code makes it so that the LEDs on the robot slowly blink between white & black when called.
    @Override
    public void execute() {

        i = i + 1;

        if (i == 12.5) {

            i = 0;

            if (currentColor == desiredColor) {
                currentColor = Color.kBlack;
            } else {
                currentColor = desiredColor;
            }
            
        }

        lights.set(currentColor);
    }

    @Override
    public void end(boolean interrupted) {
        lights.set(Color.kBlack);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
