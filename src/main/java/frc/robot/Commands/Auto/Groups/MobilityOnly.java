package frc.robot.Commands.Auto.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Auto.DriveDistance;
import frc.robot.Subsystems.DriveSystem;

public class MobilityOnly extends SequentialCommandGroup {

    public MobilityOnly(DriveSystem driveSystem) {
        addCommands(
            new DriveDistance(-2, driveSystem));
    }
}