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
    Color c = Color.kBlack;
    Color currentColor = Color.kBlack;

    public Blink(LightSubsystem lightSubsystem, Color color) {
        lights = lightSubsystem;
        c = color;
    }
    
    //This line of code makes it so that the LEDs on the robot slowly blink between white & black when called.
    @Override
    public void execute() {

        i = i + 1;

        if (i == 50) {

            i = 0;

            if (currentColor == c) {
                currentColor = Color.kBlack;
            } else {
                currentColor = c;
            }
            
        }

        lights.set(c);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
