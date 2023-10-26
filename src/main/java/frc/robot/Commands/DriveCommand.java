package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveSystem;

public class DriveCommand extends CommandBase {
    private final DriveSystem m_subsystem;

    // Init
    public DriveCommand(DriveSystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.setMotorOutput(TalonSRXControlMode.PercentOutput, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}