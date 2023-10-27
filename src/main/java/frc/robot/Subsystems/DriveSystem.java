package frc.robot.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import ca.team4308.absolutelib.wrapper.drive.TankDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;


public class DriveSystem extends TankDriveSubsystem {
        // Master Controllers
        public final TalonFX masterLeft, masterRight;

        // Slave Controllers
        private final TalonFX slaveLeft, slaveRight;

        // Controllers
        private ArrayList<TalonFX> controllersFX = new ArrayList<TalonFX>();

        // Gyro
        public static AHRS gyro = new AHRS();

        // Init
        public DriveSystem() {
                // Setup and Add Controllers
                masterLeft = new TalonFX(Constants.Mapping.Drive.frontLeft);
                controllersFX.add(masterLeft);
                masterRight = new TalonFX(Constants.Mapping.Drive.frontRight);
                controllersFX.add(masterRight);
                slaveLeft = new TalonFX(Constants.Mapping.Drive.backLeft);
                controllersFX.add(slaveLeft);
                slaveRight = new TalonFX(Constants.Mapping.Drive.backRight);
                controllersFX.add(slaveRight);
                
                // Reset Config for all
                for (TalonFX talon : controllersFX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                }

                // Set Invert Mode
                masterLeft.setInverted(TalonFXInvertType.CounterClockwise);
                masterRight.setInverted(TalonFXInvertType.Clockwise);
                slaveLeft.follow(masterLeft);
                slaveLeft.setInverted(TalonFXInvertType.FollowMaster);
                slaveRight.follow(masterRight);
                slaveRight.setInverted(TalonFXInvertType.FollowMaster);

                // Change Config For All Controllers
                for (TalonFX talon : controllersFX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                        talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp,
                                        Constants.Generic.timeoutMs);
                        talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp,
                                        Constants.Generic.timeoutMs);
                        talon.configStatorCurrentLimit(Constants.Config.Drive.Power.kStatorCurrentLimit,
                                        Constants.Generic.timeoutMs);
                        talon.setNeutralMode(NeutralMode.Coast);
                        talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
                        talon.changeMotionControlFramePeriod(5);
                        talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
                        talon.enableVoltageCompensation(true);
                }

                // Configure Primary Closed Loop Sensor
                masterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                                Constants.Generic.timeoutMs);
                masterRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                                Constants.Generic.timeoutMs);

                // Set Sensor Phase for all loops
                masterLeft.setSensorPhase(false);
                // masterRight.setSensorPhase(false);

                // Set Left Velocity PIDF values
                masterLeft.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kP, Constants.Generic.timeoutMs);
                masterLeft.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kI, Constants.Generic.timeoutMs);
                masterLeft.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kD, Constants.Generic.timeoutMs);
                masterLeft.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kF, Constants.Generic.timeoutMs);
                masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

                // Set Right Velocity PIDF values
                masterRight.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kP, Constants.Generic.timeoutMs);
                masterRight.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kI, Constants.Generic.timeoutMs);
                masterRight.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kD, Constants.Generic.timeoutMs);
                masterRight.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kF, Constants.Generic.timeoutMs);
                masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

                masterLeft.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
                masterRight.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
                masterLeft.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);
                masterRight.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);

                masterLeft.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
                masterLeft.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
                masterLeft.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
                masterLeft.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);
                masterLeft.setNeutralMode(NeutralMode.Coast);
                masterLeft.config_IntegralZone(Constants.Config.Drive.MotionMagic.profileSlot, 10);
                masterRight.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
                masterRight.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
                masterRight.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
                masterRight.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);
                masterRight.setNeutralMode(NeutralMode.Coast);
                masterRight.config_IntegralZone(Constants.Config.Drive.MotionMagic.profileSlot, 10);

                // Reset
                stopControllers();
                resetSensors();
        }

        /**
         * Getters And Setters
         */
        public double getLeftSensorPosition() {
                return masterLeft.getSelectedSensorPosition(0);
                               // * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
        }

        public double getRightSensorPosition() {
                return masterRight.getSelectedSensorPosition(0)
                                * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
        }

        public double getLeftSensorVelocity() {
                return masterLeft.getSelectedSensorVelocity(0);
        }

        public double getRightSensorVelocity() {
                return masterRight.getSelectedSensorVelocity(0);
        }

        
        public void setMotorOutput(ControlMode mode, double left, double right) {
                masterLeft.set(mode, left);
                masterRight.set(mode, right);
        }

        public void selectProfileSlot(int slot) {
                masterLeft.selectProfileSlot(slot, 0);
                masterRight.selectProfileSlot(slot, 0);
        }

        public void resetAngle() {
                gyro.reset();
        }

        public void stopControllers() {
                masterLeft.set(TalonFXControlMode.PercentOutput, 0.0);
                masterRight.set(TalonFXControlMode.PercentOutput, 0.0);
        }

        // Sensor Reset
        public void resetSensors() {
                masterLeft.setSelectedSensorPosition(0);
                masterRight.setSelectedSensorPosition(0);
        }

        @Override
        public Sendable log() {
                Shuffleboard.getTab("Log").addDouble("Angle", () -> gyro.getYaw());
                return this;
        }
}