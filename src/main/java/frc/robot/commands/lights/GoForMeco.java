// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class GoForMeco extends CommandBase {
    private final LightSubsystem lights;
    int i = 0;
    Color c = Color.kPaleGoldenrod;

    public GoForMeco(LightSubsystem lightSubsystem) {
        lights = lightSubsystem;
    }
    
    //This line of code makes it so that the LEDs on the robot slowly blink between white & black when called.
    @Override
    public void execute() {

        i = i + 1;

        if (i == 13) {

            i = 0;

            if (c == Color.kRed) {
                c = Color.kBlue;
            } else {
                c = Color.kRed;
            }
            
        }

        lights.set(c);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
