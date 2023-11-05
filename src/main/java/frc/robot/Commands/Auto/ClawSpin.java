package frc.robot.Commands.Auto;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ClawSpinSystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ClawSpin extends CommandBase {
    private final ClawSpinSystem m_subsystem;
    private final double control;

    int withinThresholdLoops;

    public ClawSpin(double control, ClawSpinSystem subsystem) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(this.m_subsystem);

        withinThresholdLoops = 0;
    }

    @Override
    public void initialize() {
        m_subsystem.motor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void execute() {
        m_subsystem.setClawSpinOutput(TalonSRXControlMode.PercentOutput, control);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}