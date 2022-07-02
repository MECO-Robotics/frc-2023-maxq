package frc.robot.subsystems;


import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("photonvision");
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    public VisionSubsystem() {
        // Set driver mode to on.
        camera.setDriverMode(true);
        // Change pipeline to 0
        camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        double forwardSpeed;
        PhotonPipelineResult result = camera.getLatestResult();
        // Capture pre-process camera stream image
        camera.takeInputSnapshot();

        // Capture post-process camera stream image
        camera.takeOutputSnapshot();
        // Check if the latest result has any targets.
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target.
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();

            System.out.println(
                    "yaw    " + "pitch    " + "area    " + "skew   " + yaw + "  " + pitch + "  " + area + "  " + skew);

        }

    }

}