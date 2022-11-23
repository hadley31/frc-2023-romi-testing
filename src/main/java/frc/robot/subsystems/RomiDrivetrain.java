// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DrivetrainPoseEstimator;
import frc.robot.ui.GlassInterface;

public class RomiDrivetrain extends SubsystemBase {
    private static final double kCountsPerRevolution = 1440.0;
    private static final double kWheelDiameterMeters = 0.070;
    private static final double kTrackWidthMeters = 0.140;

    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    private final Spark m_leftMotor = new Spark(0);
    private final Spark m_rightMotor = new Spark(1);

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);

    // Romi Gyro
    private final RomiGyro m_gyro = new RomiGyro();

    // Set up the differential drive controller
    private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private final DrivetrainPoseEstimator m_poseEstimator;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final SimpleMotorFeedforward m_leftFF = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);
    private final SimpleMotorFeedforward m_rightFF = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);

    /** Creates a new RomiDrivetrain. */
    public RomiDrivetrain() {
        m_poseEstimator = new DrivetrainPoseEstimator(this,
                PhotonCameraProvider.get(Constants.kCameraName));
        m_leftController = new PIDController(0.5, 0.0, 0.0);
        m_rightController = new PIDController(0.5, 0.0, 0.0);

        // Use meters / second as unit for encoder distances
        m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
        m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
        resetEncoders();
        resetGyroAngle();

        // Invert right side since motor is flipped
        m_rightMotor.setInverted(true);
    }

    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getLeftDistanceMeters() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistanceMeters() {
        return m_rightEncoder.getDistance();
    }

    public double getLeftRateMetersPerSecond() {
        return m_leftEncoder.getRate();
    }

    public double getRightRateMetersPerSecond() {
        return m_rightEncoder.getRate();
    }

    public void resetGyroAngle() {
        m_gyro.reset();
    }

    public Rotation2d getGyroAngle() {
        return m_gyro.getRotation2d();
    }

    public DifferentialDriveKinematics getKinematics() {
        return this.m_kinematics;
    }

    public DifferentialDriveWheelSpeeds getActualWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftRateMetersPerSecond(), getRightRateMetersPerSecond());
    }

    public void setSpeeds(double leftMetersPerSecond, double rightMetersPerSecond) {
        final double leftOutput = m_leftController.calculate(getLeftRateMetersPerSecond(), leftMetersPerSecond);
        final double rightOutput = m_rightController.calculate(getRightRateMetersPerSecond(), rightMetersPerSecond);
        m_leftMotor.setVoltage(leftOutput + m_leftFF.calculate(leftMetersPerSecond));
        m_rightMotor.setVoltage(rightOutput + m_rightFF.calculate(rightMetersPerSecond));
    }

    public void brake() {
        setSpeeds(0, 0);
    }

    public Pose2d getPose() {
        // return m_odometry.getPoseMeters();
        return m_poseEstimator.getPoseEst();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // m_odometry.update(Rotation2d.fromDegrees(-getGyroAngle()),
        // getLeftDistanceMeters(), getRightDistanceMeters());
        m_poseEstimator.update(getActualWheelSpeeds(), getLeftDistanceMeters(), getRightDistanceMeters());

        SmartDashboard.putNumber("Position Left", getLeftDistanceMeters());
        SmartDashboard.putNumber("Position Right", getRightDistanceMeters());
        SmartDashboard.putNumber("Speed Left", getLeftRateMetersPerSecond());
        SmartDashboard.putNumber("Speed Right", getRightRateMetersPerSecond());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        GlassInterface.updateRobotPose(getPose());
    }

    public void resetPoseToCameraEstimate() {
        m_poseEstimator.resetToPose(m_poseEstimator.getLastCameraPoseEst());
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetToPose(pose);
    }

}
