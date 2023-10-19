package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSystem;
import ca.team4308.absolutelib.math.DoubleUtils;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSystem m_subsystem;
    private final Supplier<Double> control;

    //needs to be updated later
    //private final PIDController extension_controller = new PIDController();
    
    // Init
    public ElevatorCommand(ElevatorSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
        //extension_controller.setSetpoint(subsystem.getSensorPosition());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double control = this.control.get();

        //needs to be updated later

        //extension_controller.setSetpoint(m_subsystem.getSensorPosition());
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, control);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
}