package frc.robot.Commands.Auto.Groups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Auto.DriveDistance;
import frc.robot.Subsystems.DriveSystem;

public class Basic extends SequentialCommandGroup {

    public Basic(DriveSystem driveSystem) {
        addCommands(
            new DriveDistance(-2, driveSystem));
    }
}