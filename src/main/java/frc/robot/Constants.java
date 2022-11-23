// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.GeometryUtils.from2dTo3d;
import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final double kDefaultLooptime = 0.02;

        public static final String kGlassFieldName = "Field";
        public static final String kGlassPPFieldName = "";

        /**
         * Name of the camera in photonvision
         * http://photonvision.local:5800
         */
        public static final String kCameraName = "camera";

        /**
         * Offset from the camera to the center of the robot (and the ground)
         */
        public static final double kCameraOffsetX = Units.inchesToMeters(1.5);
        public static final double kCameraOffsetY = Units.inchesToMeters(0);
        public static final double kCameraOffsetZ = Units.inchesToMeters(4.5);
        public static final double kCameraAngle = Units.degreesToRadians(13);
        public static final Transform3d kCameraToRobot = new Transform3d(
                        new Translation3d(-kCameraOffsetX, kCameraOffsetY, kCameraOffsetZ),
                        new Rotation3d(0, -kCameraAngle, 0)).inverse();

        /**
         * Example target position
         * See https://...
         */
        public static final double kExampleTarget0XPos = Units.inchesToMeters(100 - 34.8);
        public static final double kExampleTarget0YPos = Units.inchesToMeters(140);
        public static final double kExampleTarget0Height = Units.inchesToMeters(11.3);
        public static final Pose3d kExampleTarget0Pose = new Pose3d(
                        kExampleTarget0XPos, kExampleTarget0YPos, kExampleTarget0Height,
                        from2dTo3d(Rotation2d.fromDegrees(-90)));

        public static final double kExampleTarget1XPos = Units.inchesToMeters(100);
        public static final double kExampleTarget1YPos = Units.inchesToMeters(140);
        public static final double kExampleTargetHeight = Units.inchesToMeters(11);
        public static final Pose3d kExampleTarget1Pose = new Pose3d(
                        kExampleTarget1XPos, kExampleTarget1YPos, kExampleTargetHeight,
                        from2dTo3d(Rotation2d.fromDegrees(-90)));

        public static final double kExampleTarget2XPos = Units.inchesToMeters(143.5);
        public static final double kExampleTarget2YPos = Units.inchesToMeters(100);
        public static final double kExampleTarget2Height = Units.inchesToMeters(12);
        public static final Pose3d kExampleTarget2Pose = new Pose3d(
                        kExampleTarget2XPos, kExampleTarget2YPos, kExampleTarget2Height,
                        from2dTo3d(Rotation2d.fromDegrees(180)));

        /**
         * A mapping of fiducial ids to target positions
         */
        public static final Map<Integer, Pose3d> kFiducialPoseMapping = Map.ofEntries(
                        // entry(0, kExampleTarget0Pose),
                        entry(53, kExampleTarget0Pose),
                        entry(1, kExampleTarget1Pose),
                        entry(2, kExampleTarget2Pose));

        public static class AutoConstants {
                public static final String kTestAuto = "Test Auto";
                public static final String kAutoStatusKey = "Auto Status";

                // Auto Event Names
                public static final String kIntakeDown = "intake_down";
                public static final String kIntakeUp = "intake_up";
                public static final String kIntakeRun = "intake_run";
        }

        public static class DriveConstants {
                public static final double ksVolts = 0.929;
                public static final double kvVoltSecondsPerMeter = 6.33;
                public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

                public static final double kvAngular = 0.0389;
                public static final double kaAngular = 0.0389;

                public static final double kAutoMaxSpeedMetersPerSecond = 0.5;
                public static final double kAutoMaxAccelerationMetersPerSecondSq = 0.5;

                public static final double kCountsPerRevolution = 1440.0;
                public static final double kWheelDiameterMeters = 0.070;
                public static final double kTrackWidthMeters = 0.140;
        }
}
