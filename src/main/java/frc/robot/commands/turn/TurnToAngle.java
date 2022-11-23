package frc.robot.commands.turn;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnToAngle extends CommandBase {
    private static final double kMaxErrorRadians = Units.degreesToRadians(1.0);
    private static final double kMaxErrorRadiansPerSecond = Units.degreesToRadians(1.0);

    protected final RomiDrivetrain m_drive;
    protected final Rotation2d m_inputAngle;

    // TODO: tune PID
    protected PIDController m_controller = new PIDController(6, 0, 0.1);

    public TurnToAngle(RomiDrivetrain drive, Rotation2d angle) {
        m_drive = drive;
        m_inputAngle = angle;

        m_controller.enableContinuousInput(-Math.PI, Math.PI);
        m_controller.setTolerance(kMaxErrorRadians, kMaxErrorRadiansPerSecond);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double currentAngle = m_drive.getPose().getRotation().getRadians();
        double desiredAngleSetpoint = getDesiredAngle().getRadians();

        double rotationSpeed = m_controller.calculate(currentAngle, desiredAngleSetpoint);

        m_drive.arcadeDrive(0, rotationSpeed);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
    }

    /**
     * @return The desired angle in radians
     */
    public Rotation2d getDesiredAngle() {
        return m_inputAngle;
    }

}
