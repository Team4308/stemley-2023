package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSystem;
import ca.team4308.absolutelib.math.DoubleUtils;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSystem m_subsystem;
    private final Supplier<Double> control;
    
    // Init
    public ElevatorCommand(ElevatorSystem subsystem, Supplier<Double> control) {
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
        double control = this.control.get();

        /* 

        if (control == 0.0) {
            // stop it at current
            double output = DoubleUtils.clamp(extension_controller.calculate(m_subsystem.getSensorPosition()), -1.0,
                    1.0);
            m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, output);
            return;
        } */


   /*      if (!m_subsystem.getMinBreak()) { // If elevator is backed all the way in
            m_subsystem.motor.setSelectedSensorPosition(0);
            extension_controller.setSetpoint(m_subsystem.getSensorPosition());
            if(control > 0){
                return;
            }
        }
 */
        // this needs to be checked later
        if (!m_subsystem.getMaxBreak()) { // If elevator is all the way out
            //m_subsystem.motor.setSelectedSensorPosition();
            // extension_controller.setSetpoint(m_subsystem.getSensorPosition());
            if(control < 0){
                control = 0;
            }
        }

        // extension_controller.setSetpoint(m_subsystem.getSensorPosition());
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