/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.ui.GlassInterface;

/**
 * Performs estimation of the drivetrain's current position on the field, using
 * a vision system,
 * drivetrain encoders, and a gyroscope. These sensor readings are fused
 * together using a Kalman
 * filter. This in turn creates a best-guess at a Pose2d of where our drivetrain
 * is currently at.
 */
public class DrivetrainPoseEstimator {
    // Sensors used as part of the Pose Estimation
    private final RomiDrivetrain m_drive;
    private PhotonCamera m_camera;
    private Pose2d m_cameraEstimatedRobotPose = new Pose2d();
    // Note - drivetrain encoders are also used. The Drivetrain class must pass us
    // the relevant readings.

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your
    // various sensors. Smaller numbers will cause the filter to "trust" the
    // estimate from that particular
    // component more than the others. This in turn means the particualr component
    // will have a stronger
    // influence on the final pose estimate.
    Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
    Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
    Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.002, 0.002, Units.degreesToRadians(0.05));

    private final DifferentialDrivePoseEstimator m_poseEstimator;

    public DrivetrainPoseEstimator(RomiDrivetrain drive, PhotonCamera camera) {
        this.m_drive = drive;
        this.m_camera = camera;

        this.m_poseEstimator = new DifferentialDrivePoseEstimator(
                m_drive.getGyroAngle(),
                m_drive.getLeftDistanceMeters(),
                m_drive.getRightDistanceMeters(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01),
                VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20)));
    }

    /**
     * Perform all periodic pose estimation tasks.
     *
     * @param actWheelSpeeds Current Speeds (in m/s) of the drivetrain wheels
     * @param leftDist       Distance (in m) the left wheel has traveled
     * @param rightDist      Distance (in m) the right wheel has traveled
     */
    public void update(
            DifferentialDriveWheelSpeeds actWheelSpeeds, double leftDist, double rightDist) {
        m_poseEstimator.update(m_drive.getGyroAngle(), actWheelSpeeds, leftDist, rightDist);

        var res = m_camera.getLatestResult();
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();

            var bestTarget = res.getBestTarget();

            int fiducialId = bestTarget.getFiducialId();

            Pose3d targetPosition = Constants.kFiducialPoseMapping.get(fiducialId);

            // TODO do something if null
            if (targetPosition == null) {
                System.out.println("AHHHHHHHH could not find target position for id: " + fiducialId);
                return;
            }

            Transform3d camToTargetTransform = bestTarget.getBestCameraToTarget();
            Pose3d cameraPose = targetPosition.transformBy(camToTargetTransform.inverse());
            m_cameraEstimatedRobotPose = cameraPose.transformBy(Constants.kCameraToRobot).toPose2d();
            GlassInterface.setObjectPose("cameraPoseEstimate", m_cameraEstimatedRobotPose);

            m_poseEstimator.addVisionMeasurement(m_cameraEstimatedRobotPose, imageCaptureTime);
        }
    }

    /**
     * Force the pose estimator to a particular pose. This is useful for indicating
     * to the software
     * when you have manually moved your robot in a particular position on the field
     * (EX: when you
     * place it on the field at the start of the match).
     *
     * @param pose
     */
    public void resetToPose(Pose2d pose) {
        m_poseEstimator.resetPosition(
                m_drive.getGyroAngle(),
                m_drive.getLeftDistanceMeters(),
                m_drive.getRightDistanceMeters(),
                pose);
    }

    /** @return The current best-guess at drivetrain position on the field. */
    public Pose2d getPoseEst() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getLastCameraPoseEst() {
        return m_cameraEstimatedRobotPose;
    }
}
