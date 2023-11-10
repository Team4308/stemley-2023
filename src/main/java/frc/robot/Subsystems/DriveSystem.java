package frc.robot.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import ca.team4308.absolutelib.wrapper.drive.TankDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class DriveSystem extends MotoredSubsystem{
        // Master Controllers
        public final TalonSRX masterLeft;
        // Slave Controllers


        // Controllers
        private ArrayList<TalonSRX> controllersSRX = new ArrayList<TalonSRX>();

        // Gyro
        public static AHRS gyro = new AHRS();

        public static DigitalInput minBreak;

        public double turnRate, throttleRate;

        // Init
        public DriveSystem() {
                // Setup and Add Controllers
                masterLeft = new TalonSRX(Constants.Mapping.Drive.frontLeft);
                controllersSRX.add(masterLeft);
                minBreak = new DigitalInput(0);
                
                // Reset Config for all
                for (TalonSRX talon : controllersSRX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                }

                // Change Config For All Controllers
                for (TalonSRX talon : controllersSRX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                        talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
                        talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp, Constants.Generic.timeoutMs);
                        talon.setNeutralMode(NeutralMode.Brake);
                        talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
                        talon.changeMotionControlFramePeriod(5);
                        talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
                        talon.enableVoltageCompensation(true);
                }
                // Reset
                stopControllers();
        }

        /**
         * Getters And Setters
         */
        
        public void setMotorOutput(TalonSRXControlMode mode, double val) {
                masterLeft.set(mode, val);
        }

        public void stopControllers() {
                masterLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
        }

        public void resetAngle() {
                gyro.reset();
        }

        public boolean getMinBreak(){
                return minBreak.get();
            }

        @Override
        public Sendable log() {
                Shuffleboard.getTab("Log").addDouble("Angle", () -> gyro.getYaw());
                Shuffleboard.getTab("Log").addBoolean("Elevator Retracted", () -> getMinBreak());
                return this;
        }
}