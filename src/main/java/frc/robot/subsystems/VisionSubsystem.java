// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    NetworkTable fixedVisionCameraTable;

    DoubleSubscriber ySub;
    // use an AtomicReference to make updating the value thread-safe
    AtomicReference<Double> yValue = new AtomicReference<Double>();
    // retain listener handles for later removal
    int connListenerHandle;
    int valueListenerHandle;
    int topicListenerHandle;

    
    public class RobotPoseSample {

        public Pose2d robotPose;
        public double timeStamp;
    }

    public VisionSubsystem() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        fixedVisionCameraTable = inst.getTable("field-camera");

        // add a connection listener; the first parameter will cause the
        // callback to be called immediately for any current connections
        // connListenerHandle = inst.addConnectionListener(true, event -> {
        //     if (event.is(NetworkTableEvent.Kind.kConnected)) {
        //         System.out.println("Connected to " + event.connInfo.remote_id);
        //     } else if (event.is(NetworkTableEvent.Kind.kDisconnected)) {
        //         System.out.println("Disconnected from " + event.connInfo.remote_id);
        //     }
        // });

    }

    @Override
    public void periodic() {

    }

    RobotPoseSample getVisionMeasurement() {

        return null;
    }
}
