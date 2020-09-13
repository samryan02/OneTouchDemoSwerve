/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Driving extends Command {
  //initalize vaiable for setting set point to increase traslation speed(location)
  double[] wheelAngles;
  double[] wheelSpeeds;

  double FL1 = 0;
  double FL2 = 0;
  double FR1 = 0;
  double FR2 = 0;
  double BL1 = 0;
  double BL2 = 0;
  double BR1 = 0;
  double BR2 = 0;
  public Driving() {
    //declare substem depedency

    requires(Robot.LFmod);
    requires(Robot.LBmod);
    requires(Robot.FRmod);
    requires(Robot.BRmod);

    

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // get wheel speeds and angles from calculations class
    wheelAngles = Robot.calculations.wheelAngles();
    wheelSpeeds = Robot.calculations.wheelSpeeds();
    
    // alter the set point varible using wheel speed
    
    //integrate wheel speed and angle to setpoint for each motor
    Robot.LFmod.setModuleAngle(wheelAngles[0]);
    Robot.LFmod.setModuleSpeed(wheelSpeeds[0]);
    
    BL1 = BL1+ wheelSpeeds[3];
    BL2 = BL2- wheelSpeeds[3];
    Robot.LBmod.setModuleAngle(wheelAngles[3]);
    Robot.LBmod.setModuleSpeed(wheelSpeeds[3]);
    
    FR1 = FR1+ wheelSpeeds[1];
    FR2 = FR2- wheelSpeeds[1];
    Robot.FRmod.setModuleAngle(wheelAngles[1]);
    Robot.FRmod.setModuleSpeed(wheelSpeeds[1]);

    BR1 = BR1+ wheelSpeeds[2];
    BR2 = BR2- wheelSpeeds[2];
    Robot.BRmod.setModuleAngle(wheelAngles[2]);
    Robot.BRmod.setModuleSpeed(wheelSpeeds[2]);
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}