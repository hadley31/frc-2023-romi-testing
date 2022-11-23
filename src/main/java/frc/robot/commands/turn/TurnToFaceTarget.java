package frc.robot.commands.turn;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnToFaceTarget extends TurnToAngle {

    private Translation2d m_target;

    public TurnToFaceTarget(RomiDrivetrain drive, Translation2d target) {
        super(drive, null);
        m_target = target;
    }

    public TurnToFaceTarget(RomiDrivetrain drive, Pose2d target) {
        this(drive, target.getTranslation());
    }

    @Override
    public Rotation2d getDesiredAngle() {
        Translation2d offset = m_target.minus(m_drive.getPose().getTranslation());
        System.out.println(offset);
        return new Rotation2d(offset.getX(), offset.getY());
    }

}
