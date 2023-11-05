package frc.robot.Commands.Auto.Groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Commands.DockingCommand;

import frc.robot.Commands.Auto.DriveDistance;

import frc.robot.Subsystems.DriveSystem;


public class DockOnly extends SequentialCommandGroup {

    public DockOnly(DriveSystem driveSystem) {
        //Starts facing charging station, moves forward and docks
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> driveSystem.resetAngle(), driveSystem),
                new DriveDistance(2, driveSystem),
                new ParallelDeadlineGroup(new WaitCommand(10), new DockingCommand(driveSystem))
            )
            
        );
    }
}