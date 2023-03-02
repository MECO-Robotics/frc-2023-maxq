// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    NetworkTable limeLight;
    double latencySeconds;
    double currentTimestamp;
    Pose2d currentPose;

    public VisionSubsystem() {

        if (NetworkTableInstance.getDefault() != null) {

            limeLight = NetworkTableInstance.getDefault().getTable("limelight");

            if (limeLight != null) {

                // Get the latency. Assuming this only needs to be queried once
                double tl = limeLight.getEntry("tl").getDouble(0);
                double cl = limeLight.getEntry("cl").getDouble(0);
                latencySeconds = (tl + cl) / 1000.0;

                // Turn the lime light LED off (remove this line to let the limelight decide
                // based on the pipeline)
                limeLight.getEntry("ledMode").setNumber(1);

            } else {
                System.out.println("Can't access limelight data - is it connected?");
            }
        }
    }

    @Override
    public void periodic() {

        if (limeLight != null) {
            // Get the nex sample
            double[] sample = new double[6];

            // Get the current bot pose x,y,z in meters; roll, pitch, yaw in degrees
            limeLight.getEntry("botpose").getDoubleArray(sample);

            // Get the time at which the measurement was taken by using the current time and
            // subtracting the limelight latency
            currentTimestamp = Timer.getFPGATimestamp() - latencySeconds;

            // sample[0] is x (field forward)
            // sample[1] is y (field left)
            // sample[5] is yaw
            currentPose = new Pose2d(new Translation2d(sample[0], sample[1]),
                    new Rotation2d(Math.toRadians(sample[5])));
        }
    }

    /**
     * Get the current pose.
     * 
     * @return
     */
    public Pose2d getVisionMeasurement() {
        return currentPose;
    }

    /**
     * Get the current time stamp.
     * 
     * @return
     */
    public double getVisionMeasurementTimestamp() {
        return currentTimestamp;
    }
}
