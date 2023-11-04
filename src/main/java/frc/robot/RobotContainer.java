// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.ClawSpinCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.HoldInPlace;
import frc.robot.Commands.LEDCommand;
import frc.robot.Commands.Auto.Groups.DockOnly;
import frc.robot.Commands.Auto.Groups.MobilityOnly;
import frc.robot.Subsystems.ClawSpinSystem;
import frc.robot.Subsystems.DriveSystem;
import frc.robot.Subsystems.ElevatorSystem;
import frc.robot.Subsystems.LEDSystem;
import frc.robot.Subsystems.LimelightSystem;

public class RobotContainer {
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  //Subsystems
  private final DriveSystem m_driveSystem;
  private final ClawSpinSystem m_clawSpinSystem;
  private final ElevatorSystem m_elevatorSystem;
  private final LimelightSystem m_limelightSystem;
  private final LEDSystem m_ledSystem;

  //Commands
  private final DriveCommand driveCommand;
  private final ElevatorCommand elevatorCommand;
  private final ClawSpinCommand clawSpinCommand;
  private final LEDCommand ledCommand;
  

  //Controllers

  public final XBoxWrapper stick1 = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  private final MobilityOnly mobilityOnly;
  private final DockOnly dockOnly;

  public RobotContainer() {
    //Subsystem Instantiations
    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);
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

    clawSpinCommand = new ClawSpinCommand(m_clawSpinSystem, () -> getClawSpinControl());
    m_clawSpinSystem.setDefaultCommand(clawSpinCommand);
  
    mobilityOnly = new MobilityOnly(m_driveSystem);
    dockOnly = new DockOnly(m_driveSystem);

    autoCommandChooser.addOption("MobilityOnly", mobilityOnly);
    autoCommandChooser.addOption("DockOnly", dockOnly);

    SmartDashboard.putData(autoCommandChooser);

    configureBindings();
  }

  private void configureBindings() {
    //stick2.LB.whileTrue(new ClawSpinCommand(m_clawSpinSystem, 1));
    //stick2.RB.whileTrue(new ClawSpinCommand(m_clawSpinSystem, -1));
    stick1.B.onTrue(new InstantCommand(() -> m_driveSystem.resetAngle(), m_driveSystem));
    stick1.A.onTrue(new InstantCommand(() -> m_limelightSystem.toggleCamera(), m_limelightSystem));
    stick1.LB.whileTrue(new HoldInPlace(m_driveSystem, () -> getHoldControl()));
    stick2.RB.whileTrue(new ClawSpinCommand(m_clawSpinSystem, () -> getClawHoldControl()));
  }

  public Vector2 getDriveControl() {
    double throttle = DoubleUtils.normalize(stick1.getLeftY());

    double turn = DoubleUtils.normalize(stick1.getRightX());

    if(stick1.RB.getAsBoolean()){
      throttle /= 1.5;
      turn /= 2;
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
    if(y>0){
      control.y *= 0.15;
    }else{
      control.y *= 0.5;
    }
    // control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    // normal
    return control.y;
  }

  public Double getClawSpinControl() {
    double y;
    if(DoubleUtils.normalize(stick2.getRightY()) < 0.1 && DoubleUtils.normalize(stick2.getRightY()) > -0.1 && m_clawSpinSystem.state)y = -0.15;
    else y = DoubleUtils.normalize(stick2.getRightY()) * 0.5;
    if(y > 0.3)m_clawSpinSystem.state = false;
    if(y < -0.3)m_clawSpinSystem.state = true;
    Vector2 control = new Vector2(0.0, y);
    if(y > 0)control.y *= 0.5;
    return control.y;
  }

  public Vector2 getHoldControl(){
    Vector2 control = new Vector2(m_driveSystem.masterLeft.getSelectedSensorPosition(), m_driveSystem.masterRight.getSelectedSensorPosition());
    return control;
  }

  public Double getClawHoldControl() {
    return -0.15;
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
    // return mobilityOnly;
  }

  public Integer getLEDCommand() {
    return 1;
  }
}
