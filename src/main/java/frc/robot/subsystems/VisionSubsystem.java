// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    NetworkTable limeLight;

    public VisionSubsystem() {

        limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
    }

    /**
     * Get the 
     * @return
     */
    Pose2d getVisionMeasurement() {

        double d = Timer.getFPGATimestamp();
        // TODO: Follow these instructions:
        // https://docs.limelightvision.io/en/latest/networktables_api.html
        // 1. Read the botpose entry from the lime light
        // 2. Move the values into a Pose2d data type
        // 3. Return the pose
        return null;
    }
}
