package frc.robot.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class ElevatorSystem extends MotoredSubsystem {
    public final TalonFX motor;

    private ArrayList<TalonFX> controllersFX = new ArrayList<TalonFX>();

    public ElevatorSystem() {
        // Setup and Add Controllers

        //needs to be updated later
        motor = new TalonFX(Constants.Mapping.Elevator.elevatorMotor);

        controllersFX.add(motor);

        // Reset Config for all
        for (TalonFX talon : controllersFX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Change Config For All Controllers
        for (TalonFX talon : controllersFX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
            talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp, Constants.Generic.timeoutMs);
            talon.setNeutralMode(NeutralMode.Brake);
            talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            talon.changeMotionControlFramePeriod(5);
            talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            talon.enableVoltageCompensation(true);
        }

        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Generic.timeoutMs);

        // Reset
        resetSensors();
        stopControllers();
    }

    public void resetSensors() {
        motor.setSelectedSensorPosition(0);
    }

    /**
     * Getters And Setters
     */

    /**
     * Misc Stuff
     */

    public double getSensorPosition() {
        return motor.getSelectedSensorPosition(0);
    }

    public void setMotorOutput(TalonFXControlMode mode, double val) {
        motor.set(mode, val);
    }

    public void stopControllers() {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }


    @Override
    public Sendable log() {
        return this;
    }
}