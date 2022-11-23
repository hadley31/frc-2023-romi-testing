package frc.robot.commands.turn;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnByAngle extends TurnToAngle {

    private Rotation2d m_desiredAngle;

    public TurnByAngle(RomiDrivetrain drive, Rotation2d angle) {
        super(drive, angle);
    }

    @Override
    public void initialize() {
        m_desiredAngle = m_drive.getPose().getRotation().plus(m_inputAngle);
    }

    @Override
    public Rotation2d getDesiredAngle() {
        return m_desiredAngle;
    }

}
