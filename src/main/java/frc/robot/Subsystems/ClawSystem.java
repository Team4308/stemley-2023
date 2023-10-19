package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;

import ca.team4308.absolutelib.wrapper.LogSubsystem;

public class ClawSystem extends LogSubsystem {
    public final DoubleSolenoid solenoid;

    public ClawSystem() {
        solenoid = new DoubleSolenoid();
        solenoid.set(Value.kForward); // in
    }

    /**
     * Getters And Setters
     */

    public void toggle() {
        if (solenoid.get() == Value.kForward) {
            solenoid.set(Value.kReverse);
        } else {
            solenoid.set(Value.kForward);
        }

    }

    @Override
    public Sendable log() {
        return this;
    }
}