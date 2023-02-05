// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

    PWM r = new PWM(0);
    PWM g = new PWM(1);
    PWM b = new PWM(2);

    public LightSubsystem() {

        // The idea here is to set the max value (1.0) to be 100% duty cycle - all on,
        // and 0 to be 0% duty cycle - all off.
        // The timer runs every 5.005 ms, so 5.005 is the largest width possible.
        r.setBounds(5.005, 0, 0, 0, 0);
        g.setBounds(5.005, 0, 0, 0, 0);
        b.setBounds(5.005, 0, 0, 0, 0);
    }

    /**
     * Set the color of the strip. Each color can have a value from 0.0 to 1.0.
     * 
     * @param red
     * @param green
     * @param blue
     */
    public void set(double red, double green, double blue) {
        r.setPosition(red);
        g.setPosition(green);
        b.setPosition(blue);
    }

}
