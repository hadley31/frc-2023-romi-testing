package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.ui.GlassInterface;

public final class TrajectoryFactory {

    private TrajectoryFactory() {

    }

    public static Command getTrajectoryCommand(RomiDrivetrain drive) {
        List<Pose2d> waypoints = List.of(
                new Pose2d(),
                new Pose2d(Units.feetToMeters(8), Units.feetToMeters(0), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.feetToMeters(11), Units.feetToMeters(1.5), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.feetToMeters(14), Units.feetToMeters(-1.5), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.feetToMeters(16), Units.feetToMeters(-1.5), Rotation2d.fromDegrees(30)),
                new Pose2d(Units.feetToMeters(17), Units.feetToMeters(0), Rotation2d.fromDegrees(90)),
                new Pose2d(Units.feetToMeters(15), Units.feetToMeters(1.5), Rotation2d.fromDegrees(180)),
                new Pose2d(Units.feetToMeters(14), Units.feetToMeters(1.5), Rotation2d.fromDegrees(180)),
                new Pose2d(Units.feetToMeters(10), Units.feetToMeters(0), Rotation2d.fromDegrees(200)),
                new Pose2d(Units.feetToMeters(6), Units.feetToMeters(0), Rotation2d.fromDegrees(180)),
                new Pose2d(Units.feetToMeters(6), Units.feetToMeters(0), Rotation2d.fromDegrees(0)));

        double maxVelocityMetersPerSecond = 0.3;
        double maxAccelerationMetersPerSecondSq = 0.2;

        TrajectoryConfig config = new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
        config.setKinematics(drive.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
        RamseteController controller = new RamseteController();

        // Push the trajectory to Field2d.
        GlassInterface.setTrajectory("trag", trajectory);

        return new RamseteCommand(trajectory, drive::getPose, controller, drive.getKinematics(), drive::setSpeeds,
                drive).andThen(() -> drive.brake(), drive);
    }
}
