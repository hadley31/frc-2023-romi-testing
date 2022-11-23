package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

public class PhotonCameraProvider {
    private static Map<String, PhotonCamera> cameras = new HashMap<>();

    private PhotonCameraProvider() {

    }

    public static PhotonCamera get(String cameraName) {
        // if (RobotBase.isSimulation()) {
        //     System.out.println("SIMULATION");
        //     var cam = new SimPhotonCamera(cameraName);

        //     cam.submitProcessedFrame(0.02);

        //     return cam;
        // }

        System.out.println("NOT SIMULATION");

        if (cameras.get(cameraName) == null) {
            var camera = new PhotonCamera(CameraNetworkTableInstance.getDefault(), cameraName);
            cameras.put(cameraName, camera);
        }

        return cameras.get(cameraName);
    }
}
