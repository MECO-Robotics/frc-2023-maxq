
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LightSubsystem extends SubsystemBase {

    private PWM channelRedLeft = new PWM(0);
    private PWM channelGreenLeft = new PWM(1);
    private PWM channelBlueLeft = new PWM(2);
    private PWM channelRedRight = new PWM(3);
    private PWM channelGreenRight = new PWM(4);
    private PWM channelBlueRight = new PWM(5);







    public LightSubsystem() {

    }

    

    /**
     * turn off all color and turn on red only on left side
     * 
     * @param brightness 0-100
     */
    public void turnRedLeftOn(int brightness) {
        channelRedLeft.setRaw(brightness);
    }

    /**
     * turn off all color and turn on red only on right side
     * 
     * @param brightness 0-100
     */
    public void turnRedRightOn(int brightness) {
        channelRedRight.setRaw(brightness);
    }

    /**
     * turn off all color and turn on blue only on left side
     * 
     * @param brightness 0-100
     */
    public void turnBlueLeftOn(int brightness) {
        channelBlueLeft.setRaw(brightness);
    }

    /**
     * turn off all color and turn on blue only on right side
     * 
     * @param brightness 0-100
     */
    public void turnBlueRightOn(int brightness) {
        channelBlueRight.setRaw(brightness);
    }

    public void turnLeftOff() {
        channelRedLeft.setRaw(0);
        channelBlueLeft.setRaw(0);
        channelGreenLeft.setRaw(0);
    }

    public void turnRightOff() {
        channelRedRight.setRaw(0);
        channelBlueRight.setRaw(0);
        channelGreenRight.setRaw(0);
    }















}
