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

import frc.robot.commands.DriveCommand;

public class RobotContainer {
  public final List<LogSubsystem> subsystems. new ArrayList<LogSubsystem>();

  //Subsystems
  private final DriveSystem m_driveSystem;

  //Commands
  private final DriveCommand driveCommand;

  //Controllers

  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  public RobotContainer() {
    //Subsystem Instantiations
    m_driveSystem = new DriveSystem();

    //Command Instantiations
    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl())
    m_driveSystem.setDefaultCommand(driveCommand);

    configureBindings();
  }

  private void configureBindings() {

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
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
    // test
  }
}
