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

import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.DriveSystem;


public class RobotContainer {
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  //Subsystems
  private final DriveSystem m_driveSystem;

  //Commands

  //Controllers

  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  public RobotContainer() {
    //Subsystem Instantiations
    m_driveSystem = new DriveSystem();

    //Command Instantiations

    configureBindings();
  }

  private void configureBindings() {
    stick.LB.whileTrue(new DriveCommand(m_driveSystem));
  }

  public Vector2 getDriveControl() {
    double throttle = DoubleUtils.normalize(stick.getLeftY());
    if(stick.RB.getAsBoolean()){
      //throttle /= 2;
    }

    double turn = DoubleUtils.normalize(stick.getRightX());
    if(stick.getLeftY()!=0.0){
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

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
    // test
  }
}
