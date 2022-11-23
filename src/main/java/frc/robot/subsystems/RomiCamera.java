package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiCamera extends SubsystemBase {
    private PhotonCamera camera;

    private PhotonTrackedTarget latestResult;

    public RomiCamera(String cameraName) {
        this.camera = PhotonCameraProvider.get(cameraName);
        // this.camera = new PhotonCamera(cameraName);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            this.latestResult = null;
            return;
        }

        var bestTarget = result.getBestTarget();

        // System.out.println("Fiducial Id: " + bestTarget.getFiducialId());

        this.latestResult = bestTarget;
    }

    public PhotonTrackedTarget getLatestResult() {
        return this.latestResult;
    }

}
