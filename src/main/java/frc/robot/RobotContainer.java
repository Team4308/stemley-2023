// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;

import frc.robot.Subsystems.DriveSystem;
import frc.robot.Subsystems.ClawSystem;
import frc.robot.Subsystems.ClawSpinSystem;
import frc.robot.Subsystems.ElevatorSystem;
import frc.robot.Subsystems.LimelightSystem;
import frc.robot.Subsystems.LEDSystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// andrew dai was here :3

import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.ClawSpinCommand;
import frc.robot.Commands.LEDCommand;

import frc.robot.Commands.Auto.DriveDistance;
import frc.robot.Commands.Auto.Groups.Basic;

public class RobotContainer {
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  //Subsystems
  private final DriveSystem m_driveSystem;
  private final ClawSystem m_clawSystem;
  private final ClawSpinSystem m_clawSpinSystem;
  private final ElevatorSystem m_elevatorSystem;
  private final LimelightSystem m_limelightSystem;
  private final LEDSystem m_ledSystem;

  //Commands
  private final DriveCommand driveCommand;
  private final ElevatorCommand elevatorCommand;
  private final LEDCommand ledCommand;

  //Controllers

  public final XBoxWrapper stick1 = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  private final Basic basic;

  public RobotContainer() {
    //Subsystem Instantiations
    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);
    m_clawSystem = new ClawSystem();
    subsystems.add(m_clawSystem);
    m_clawSpinSystem = new ClawSpinSystem();
    subsystems.add(m_clawSpinSystem);
    m_elevatorSystem = new ElevatorSystem();
    subsystems.add(m_elevatorSystem);
    m_limelightSystem = new LimelightSystem();
    subsystems.add(m_limelightSystem);
    m_ledSystem = new LEDSystem();
    subsystems.add(m_ledSystem);

    //Command Instantiations
    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
    m_driveSystem.setDefaultCommand(driveCommand);

    elevatorCommand = new ElevatorCommand(m_elevatorSystem, () -> getElevatorControl());
    m_elevatorSystem.setDefaultCommand(elevatorCommand);

    ledCommand = new LEDCommand(m_ledSystem, () -> getLEDCommand());
    m_ledSystem.setDefaultCommand(ledCommand);
  
    basic = new Basic(m_driveSystem);

    autoCommandChooser.addOption("basic", basic);

    configureBindings();
  }

  private void configureBindings() {
    stick2.LB.whileTrue(new ClawSpinCommand(m_clawSpinSystem, 1));
    stick2.RB.whileTrue(new ClawSpinCommand(m_clawSpinSystem, -1));
    stick1.A.onTrue(new InstantCommand(() -> m_limelightSystem.toggleCamera(), m_limelightSystem));
  }

  public Vector2 getDriveControl() {
    double throttle = DoubleUtils.normalize(stick1.getLeftY());
    if(stick1.RB.getAsBoolean()){
      //throttle /= 2;
    }

    double turn = DoubleUtils.normalize(stick1.getRightX());
    if(stick1.getLeftY()!=0.0){
        //increase turn in here
        //turn *= 1.4;
    }
    else{
      //turn -= throttle*0.4;
    }

    Vector2 control = new Vector2(turn, throttle);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.scaleStick(control, Constants.Config.Input.Stick.kInputScale);
    control = JoystickHelper.clampStick(control);

    return control;
  }

  public Double getElevatorControl() {
    double y = DoubleUtils.normalize(stick2.getLeftY());
    Vector2 control = new Vector2(0.0, y);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.clampStick(control);
    return control.y;
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }

  public Integer getLEDCommand() {
    return 1;
  }
}
