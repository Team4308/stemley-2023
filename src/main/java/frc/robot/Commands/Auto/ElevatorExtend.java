package frc.robot.Commands.Auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSystem;

public class ElevatorExtend extends CommandBase {
    private final ElevatorSystem m_subsystem;
    private final double control;

    int withinThresholdLoops;

    public ElevatorExtend(double control, ElevatorSystem subsystem) {
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
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, control);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, 0);
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}