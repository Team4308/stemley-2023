package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.DriveSystem;

public class DriveCommand extends CommandBase {
    private final DriveSystem m_subsystem;
    private final double control;

    // Init
    public DriveCommand(DriveSystem subsystem, double control) {
        m_subsystem = subsystem;
        this.control = control;
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
        double control = this.control;
        m_subsystem.setMotorOutput(TalonSRXControlMode.PercentOutput, control);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}