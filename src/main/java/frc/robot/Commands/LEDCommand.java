package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LEDSystem;

public class LEDCommand extends CommandBase {
    private final LEDSystem m_subsystem;
    private final Supplier<Integer> control;
    private int currentControl = 999;

    // Init
    public LEDCommand(LEDSystem subsystem, Supplier<Integer> control) {
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
        int control = this.control.get();
        if (currentControl == control) return; // Prevents unnecessary calls to setRGB
        currentControl = control;
        
        switch (control){
            case 1: // default red always
                m_subsystem.setRGB(255,0,0); // red
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

}
