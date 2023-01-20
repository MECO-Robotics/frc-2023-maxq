package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    public class RobotPoseSample{
        public Pose2d robotPose;
        public double timeStamp;
    }

    @Override
    public void periodic() {

    }

    RobotPoseSample getVisionMeasurement() {
        
        return null;
    }
}
