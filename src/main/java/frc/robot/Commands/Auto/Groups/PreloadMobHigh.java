package frc.robot.Commands.Auto.Groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.ClawSpinCommand;
import frc.robot.Commands.DockingCommand;
import frc.robot.Commands.Auto.DriveDistance;
import frc.robot.Commands.Auto.ElevatorExtend;
import frc.robot.Commands.Auto.ClawSpin;
import frc.robot.Subsystems.DriveSystem;
import frc.robot.Subsystems.ElevatorSystem;
import frc.robot.Subsystems.ClawSpinSystem;

public class PreloadMobHigh extends SequentialCommandGroup {

    public PreloadMobHigh(DriveSystem driveSystem, ElevatorSystem elevatorSystem, ClawSpinSystem clawSpinSystem) {
        addCommands(
            //places game piece on high node, skips docking, then passes mobility bonus line

            //game piece
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(1), new ElevatorExtend(-1, elevatorSystem)),
                new ParallelDeadlineGroup(new WaitCommand(1), new ClawSpin(-1, clawSpinSystem)),
                new ParallelDeadlineGroup(new WaitCommand(1), new ElevatorExtend(1, elevatorSystem)),
                new DriveDistance(-2, driveSystem)
            )
        );
    }
}