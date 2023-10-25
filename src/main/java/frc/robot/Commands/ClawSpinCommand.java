package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ClawSpinSystem;

public class ClawSpinCommand extends CommandBase {
    private final ClawSpinSystem m_subsystem;
    private final Supplier<Double> control;

    // Init
    public ClawSpinCommand(ClawSpinSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double control = this.control.get();

        m_subsystem.setClawSpinOutput(TalonSRXControlMode.PercentOutput, control);
    }
    @Override
    public boolean isFinished() {return false;}
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}