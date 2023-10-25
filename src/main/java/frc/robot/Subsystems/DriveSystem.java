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

public class DriveSystem extends MotoredSubsystem{
        // Master Controllers
        public final TalonSRX masterLeft;
        // Slave Controllers


        // Controllers
        private ArrayList<TalonSRX> controllersSRX = new ArrayList<TalonSRX>();

        // Init
        public DriveSystem() {
                // Setup and Add Controllers
                masterLeft = new TalonSRX(Constants.Mapping.Drive.frontLeft);
                controllersSRX.add(masterLeft);
                
                // Reset Config for all
                for (TalonSRX talon : controllersSRX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                }

                // Change Config For All Controllers
                for (TalonSRX talon : controllersSRX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                        talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp,
                                        Constants.Generic.timeoutMs);
                        talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp,
                                        Constants.Generic.timeoutMs);
                        talon.setNeutralMode(NeutralMode.Coast);
                        talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
                        talon.changeMotionControlFramePeriod(5);
                        talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
                        talon.enableVoltageCompensation(true);
                }

                // Set Sensor Phase for all loops
                masterLeft.setSensorPhase(false);
                // masterRight.setSensorPhase(false);


                // Reset
                stopControllers();
        }

        /**
         * Getters And Setters
         */
        public double getLeftSensorPosition() {
                return masterLeft.getSelectedSensorPosition(0);
                               // * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
        }

        public double getRightSensorPosition() {
                return 0;
        }

        public double getLeftSensorVelocity() {
                return masterLeft.getSelectedSensorVelocity(0);
        }

        public double getRightSensorVelocity() {
                return 0;
        }

        
        public void setMotorOutput() {
                masterLeft.set(TalonSRXControlMode.PercentOutput, 0.5);
        }

        public void selectProfileSlot(int slot) {
                masterLeft.selectProfileSlot(slot, 0);
        }


        public void stopControllers() {
                masterLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
        }

        @Override
        public Sendable log() {
                return this;
        }
}