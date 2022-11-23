package frc.robot.ui;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class GlassInterface {
    private static final Field2d field = new Field2d();

    static {
        SmartDashboard.putData(Constants.kGlassFieldName, field);

        var targetPoses = Constants.kFiducialPoseMapping.values().stream().map(x -> x.toPose2d())
                .toArray(Pose2d[]::new);
        setObjectPoses("exampleTargets", targetPoses);
    }

    public static void setObjects(Map<String, Pose2d> objects) {
        for (var entry : objects.entrySet()) {
            String objectName = entry.getKey();
            Pose2d objectPose = entry.getValue();
            field.getObject(objectName).setPose(objectPose);
        }
    }

    public static void setObjectPoses(String name, Pose2d... poses) {
        field.getObject(name).setPoses(poses);
    }

    public static Pose2d getObjectPose(String name) {
        return field.getObject(name).getPose();
    }

    public static void setObjectPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }

    public static void setTrajectory(String name, Trajectory trajectory) {
        field.getObject(name).setTrajectory(trajectory);
    }

    public static void updateRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

}
