package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraNetworkTableInstance {
    private static NetworkTableInstance instance;

    private static final String SERVER_NAME = "10.0.0.2";
    private static final int PORT = 1735;

    private CameraNetworkTableInstance() {

    }

    public static NetworkTableInstance getDefault() {
        if (instance != null && instance.isConnected()) {
            return instance;
        }

        System.out.println("Creating network table instance...");
        instance = NetworkTableInstance.create();

        System.out.println("Starting network table client...");
        instance.setServer(SERVER_NAME, PORT);
        instance.startClient3("client");

        System.out.println("Sleeping for one second...");
        sleep(1000);

        System.out.println("Testing connection status...");
        boolean connected = instance.isConnected();

        String connectionStatusString = connected ? "is connected!" : "was unable to connect!";
        System.out.println("Network table instance " + connectionStatusString);

        return instance;
    }

    private static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }
}
