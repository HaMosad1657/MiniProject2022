package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class Photon {

    PhotonCamera camera = new PhotonCamera("MicrosoftCamera");

    public double robotCurrentX() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            return bestCameraToTarget.getX();
        }
        return 0.0;
    }

    public double robotCurrentY() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            return bestCameraToTarget.getY();
        }
        return 0.0;
    }

    public double robotCurrentZ() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            return bestCameraToTarget.getZ();
        }
        return 0.0;
    }
}
