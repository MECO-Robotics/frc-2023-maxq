package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;

/**
 * Filtered range sensor that uses a Ultrasonic sensor (MB1013) filtered through a median filter,
 * to remove outliers.
 * 
 * The MB1013 ultrasonic sensor automatically pings and reads a range every 100ms.
 */
public class RangeSensor implements Sendable {

    // The ultrasonic sensor (https://www.andymark.com/products/ultrasonic-proximity-sensor-ez-mb1013-maxbotix?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6OkNhdGFsb2c6OkNhdGVnb3J5LzVlZmUxZGI1MDNmNTcxM2M2OTViNDAwZA)
    // is connected to the RoboRio via Analog input. This gives us a 0-5v value, with
    // each 0.005 volts indicating 5mm distance.
    private final AnalogInput ultrasonicSensor;

    private final AnalogInputSim ultrasonicSensorSim;

    // The raw ultrasonic sensor sample is run through a MedianFilter to 
    // eliminate outliers. Assumes the getRange() is called within a 
    // periodic loop, so every 20ms. This finds the median over a period
    // of 100 ms. Traveling at slow speed of .25 m/s, the range of samples
    // over 0.1 seconds is 0.025m or 25 mm, which is 1 inch.
    private final MedianFilter filter = new MedianFilter(5);
    

    public RangeSensor(int analogChannel) {
        ultrasonicSensor = new AnalogInput(analogChannel);
        ultrasonicSensorSim = new AnalogInputSim(ultrasonicSensor);
    }

    /**
     * When running in simulator mode, allow setting the voltage for the sample.
     * @param voltage
     */
    public void setSensorSim(double voltage) {
        ultrasonicSensorSim.setVoltage(voltage);
    }

    /**
     * Get range, in millimeters. It's expected that this method is 
     * called repeatedly at a regular interval to allow consistent
     * results from the filtering. i.e. if you only call this once during
     * an autonomous command, you might not have enough samples to eliminate
     * the noise.
     * 
     * @return Current range to closest object, in millimeters.
     */
    public double getRange() {

        // The raw voltage is used with no oversampling or averaging. For 
        // Ultrasonics, it's better to pass through a median filter to 
        // remove outlier samples.
        double sample = filter.calculate(ultrasonicSensor.getVoltage());

        // The calculation is:
        //
        //         (analog sample in volts)   1024 sample increments          5mm
        // RANGE =                          X ----------------------  X ----------------
        //                                          5 volts             sample increment
        //
        // which, if you cancel out the units and simplify reduces to
        //
        // RANGE = (analog sample in volts) X 1024
        return sample * 1024.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Range", this::getRange, null);
    }
}
