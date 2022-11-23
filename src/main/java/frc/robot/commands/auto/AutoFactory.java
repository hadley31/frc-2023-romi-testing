package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.debug.PutSmartDashboardValue;
import frc.robot.subsystems.RomiDrivetrain;

public class AutoFactory {

    public static CommandBase testAuto(RomiDrivetrain drive) {

        // Define PathPlanner Event Map
        HashMap<String, Command> eventMap = new HashMap<String, Command>(Map.of(
                AutoConstants.kIntakeDown, new PutSmartDashboardValue(AutoConstants.kAutoStatusKey, "Intake down"),
                AutoConstants.kIntakeUp, new PutSmartDashboardValue(AutoConstants.kAutoStatusKey, "Intake up"),
                AutoConstants.kIntakeRun, new PutSmartDashboardValue(AutoConstants.kAutoStatusKey, "Run Intake"),
                "end", new PutSmartDashboardValue(AutoConstants.kAutoStatusKey, "Auto Complate"),
                "test5", new PutSmartDashboardValue(AutoConstants.kAutoStatusKey, "Test 5"),
                "test4", new PutSmartDashboardValue(AutoConstants.kAutoStatusKey, "Test 4")));

        // Load paths from pathplanner
        var paths = PathPlanner.loadPathGroup(AutoConstants.kTestAuto,
                DriveConstants.kAutoMaxSpeedMetersPerSecond,
                DriveConstants.kAutoMaxAccelerationMetersPerSecondSq);

        RamseteController controller = new RamseteController();

        // Convert paths to path follower commands
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                drive::getPose,
                drive::resetPose,
                controller,
                drive.getKinematics(),
                drive::setSpeeds,
                eventMap,
                drive);

        return autoBuilder.fullAuto(paths).andThen(() -> drive.brake());
    }
}
