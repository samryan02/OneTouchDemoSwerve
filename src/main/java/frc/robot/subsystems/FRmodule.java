/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Driving;



public class FRmodule extends Subsystem {
  private CANSparkMax FrontRightNeo1;
  private CANSparkMax FrontRightNeo2;
  private CANPIDController pidControllerFR1;
  private CANPIDController pidControllerFR2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public FRmodule() {
    FrontRightNeo1 = new CANSparkMax(3, MotorType.kBrushless);
    FrontRightNeo2 = new CANSparkMax(4, MotorType.kBrushless);
    pidControllerFR1 = FrontRightNeo1.getPIDController();
    pidControllerFR2 = FrontRightNeo2.getPIDController();
    FrontRightNeo1.getEncoder();
    FrontRightNeo2.getEncoder();
    kP = 9e-2; 
    kI = 5e-4;
    kD = 6e-7; 
    kIz = .7; 
    kFF = 0.000015; 
    kMaxOutput = .5; 
    kMinOutput = -.5;
    maxRPM = 500;

    pidControllerFR1.setP(kP);
    pidControllerFR1.setI(kI);
    pidControllerFR1.setD(kD);
    pidControllerFR1.setIZone(kIz);
    pidControllerFR1.setFF(kFF);
    pidControllerFR1.setOutputRange(kMinOutput, kMaxOutput);

    pidControllerFR2.setP(kP);
    pidControllerFR2.setI(kI);
    pidControllerFR2.setD(kD);
    pidControllerFR2.setIZone(kIz);
    pidControllerFR2.setFF(kFF);
    pidControllerFR2.setOutputRange(kMinOutput, kMaxOutput);
    // set PID coefficients
    
  }  
  public void setModuleAngle(double setPoint) {

    pidControllerFR1.setReference(setPoint, ControlType.kPosition);

  }

  public void setModuleSpeed(double setPoint) {

    FrontRightNeo2.set(setPoint);


  }

  public double getModuleAngle() {
    double first = FrontRightNeo1.getEncoder().getPosition();
    return first*60;
  }

  public double getWheelAngle() {
    double second = FrontRightNeo2.getEncoder().getPosition();
    return second/9;
  }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Driving());
  }
}
