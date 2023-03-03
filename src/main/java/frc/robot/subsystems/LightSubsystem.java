// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightSubsystem extends SubsystemBase {

    PWM r = new PWM(Constants.LIGHTS_R_PWM);
    PWM g = new PWM(Constants.LIGHTS_G_PWM);
    PWM b = new PWM(Constants.LIGHTS_B_PWM);

    public LightSubsystem() {

        // The idea here is to set the max value (1.0) to be 100% duty cycle - all on,
        // and 0 to be 0% duty cycle - all off.
        // The timer runs every 5.005 ms, so 5.005 is the largest width possible.
        // Setting the center pulse width to zero ensures
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
    public void set(Color color) {
        r.setSpeed(color.red);
        g.setSpeed(color.green);
        b.setSpeed(color.blue);
    }

    // THE CODE BELOW THIS LINE IS AN ALTERNATE APPROACH IF WE HAVE TO USE DIGITAL
    // I/O

    private static final int CYCLES = 10;

    class DutyCycle {
        long flip = 0;
        long count = 0;

        public DutyCycle(long flipIn) {
            flip = flipIn;
        }

        public boolean get() {
            if (flip == 0) {
                return false;
            } else if (flip == CYCLES) {
                return true;
            } else if (count % flip == 0) {
                count++;
                return false;
            } else {
                count++;
                return true;
            }
        }

    }

    /*
    DutyCycle red = new DutyCycle(0), green = new DutyCycle(0), blue = new DutyCycle(0);
    DigitalOutput redOut = new DigitalOutput(0), greenOut = new DigitalOutput(0), blueOut = new DigitalOutput(0);

    public void setDigital(Color color) {
        red = new DutyCycle(Math.round(color.red * CYCLES));
        green = new DutyCycle(Math.round(color.green * CYCLES));
        blue = new DutyCycle(Math.round(color.blue * CYCLES));
    }

    @Override
    public void periodic() {
        redOut.set(red.get());
        greenOut.set(green.get());
        blueOut.set(blue.get());
    }
    */
}
