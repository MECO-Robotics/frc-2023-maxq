package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

/**
 * This class monitors stator output from the motor contorller to detect
 * a stall condition.
 */
public class MotorStallMonitor {

    private final TalonSRX motorController;
    private int highCurrentDrawCounter = 0;

    // Track the last few samples to allow finding a rolling average of current draw.
    // At 20ms intervals between samples, 5 samples is 1/10 second worth of data.
    private double[] currentDrawSamples = new double[] { 0,0,0,0,0 };
    private double currentDrawSampleSum = 0;
    private int currentDrawSampleIdx = 0;

    public MotorStallMonitor(TalonSRX controller) {
        motorController = controller;
    }

    /**
     * Call with each robot periodic cycle - every 20ms
     */
    public void periodic() {

        // adjust the sum of the last few samples and replace the sample value at the 
        // current index
        currentDrawSampleSum -= currentDrawSamples[currentDrawSampleIdx];
        currentDrawSamples[currentDrawSampleIdx] = motorController.getStatorCurrent();
        currentDrawSampleSum += currentDrawSamples[currentDrawSampleIdx];
        double aveDraw = currentDrawSampleSum / (float)currentDrawSamples.length;
        // this changes the values like this:  0..1..2..3..4..0..1..2..3..
        // it allows us to loop through and keep track of 5 items, always replacing the oldest
        // with the newest. In data structures, this is called a Single Ended Queue. 
        currentDrawSampleIdx = (currentDrawSampleIdx + 1) % currentDrawSamples.length;

        // Track how many times the current remains above a current threshold 
        if(aveDraw > Constants.HIGH_CURRENT_THRESHOLD) {
            highCurrentDrawCounter++;
        } else {
            highCurrentDrawCounter = 0;
        }
    }
    
    public boolean isStalled() {
        // Stall condition is when we've had high current draw for 500 ms
        // 500 ms / 20 ms per periodic = 25 iterations
        return highCurrentDrawCounter > 25;
    }
}
