
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.lights.TurnRedOffBoth;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class LightSubsystem extends SubsystemBase {

    private int flashCount = 0;

    private final static int DIO_OFFSET = 4;
    private DigitalOutput redLeft = new DigitalOutput(0 + DIO_OFFSET);
    private DigitalOutput greenLeft = new DigitalOutput(1 + DIO_OFFSET);
    private DigitalOutput blueLeft = new DigitalOutput(2 + DIO_OFFSET);
    private DigitalOutput redRight = new DigitalOutput(3 + DIO_OFFSET);
    private DigitalOutput greenRight = new DigitalOutput(4 + DIO_OFFSET);
    private DigitalOutput blueRight = new DigitalOutput(5 + DIO_OFFSET);
    public void turBothOnRedFull() {
        redLeft.set(false);
        redRight.set(false);

    }


    
    public void turnBothOffRedFull() {
        redLeft.set(true);
        redRight.set(true);

    }

    public void turnBothOnRed(int brightness) {
        redLeft.set(false);
        redRight.set(false);
    }

    public void tunrBothOnBlue(int brightness) {
        blueLeft.set(false);
        blueRight.set(false);
    }

    /**
     * turn off all color and turn on red only on left side
     * 
     * @param brightness 0-100
     */
    public void turnRedLeftOn(int brightness) {
        redLeft.set(false);

    }

    /**
     * turn off all color and turn on red only on right side
     * 
     * @param brightness 0-100
     */
    public void turnRedRightOn(int brightness) {
        redRight.set(false);
    }

    /**
     * turn off all color and turn on blue only on left side
     * 
     * @param brightness 0-100
     */
    public void turnBlueLeftOn(int brightness) {
        blueLeft.set(false);
    }

    /**
     * turn off all color and turn on blue only on right side
     * 
     * @param brightness 0-100
     */
    public void turnBlueRightOn(int brightness) {
        blueRight.set(false);
    }

    public void turnLeftOff() {
        redLeft.set(true);
        blueLeft.set(true);
        greenLeft.set(true);

    }

    public void turnRightOff() {
        redRight.set(true);
        blueRight.set(true);
        greenRight.set(true);
    }

    public void TurBlueBothOn() {
        blueRight.set(false);
        blueLeft.set(false);
    }

    public void turnBlueOffBoth() {
        blueRight.set(true);
        blueLeft.set(true);
    }

    public void turnGreenBothOn() {
    greenLeft.set(false);
    greenRight.set(false);
    }

    public void turnGreenBothOff() {
    greenLeft.set(true);
    greenRight.set(true);
    }


    public void turnPurpleRightOn() {
        redRight.set(false);
        blueRight.set(false);  
    }


    public void turnPurpleRightOff() {
        redRight.set(true);
        blueRight.set(true);
    }



    public void turnBlueLeftOn() {
    blueLeft.set(false);
    }


    
     public void strobe(){
        flashCount = flashCount + 1;
        if(flashCount == 50){
             flashCount = 0;
        } else{
            if(flashCount <25){
                //turn red
                turnBlueOffBoth();
                turnBothOnRed(255);
            }else{
                //between 25 and 50 turn blue
                turnBothOffRedFull();
                tunrBothOnBlue(255);
            }
        }
        
    
    
    }


}
