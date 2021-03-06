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

// please note that the LF mod class is fully commented
public class BRmodule extends Subsystem {
  private CANSparkMax BackRightNeo1;
  private CANSparkMax BackRightNeo2;
  private CANPIDController pidControllerBR1;
  private CANPIDController pidControllerBR2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public BRmodule() {
    BackRightNeo1 = new CANSparkMax(7, MotorType.kBrushless);
    BackRightNeo2 = new CANSparkMax(8, MotorType.kBrushless);
    pidControllerBR1 = BackRightNeo1.getPIDController();
    pidControllerBR2 = BackRightNeo2.getPIDController();
    BackRightNeo1.getEncoder();
    BackRightNeo2.getEncoder();
    kP = 9e-2;
    kI = 5e-4;
    kD = 6e-7;
    kIz = .7;
    kFF = 0.000015;
    kMaxOutput = .5;
    kMinOutput = -.5;
    maxRPM = 500;

    pidControllerBR1.setP(kP);
    pidControllerBR1.setI(kI);
    pidControllerBR1.setD(kD);
    pidControllerBR1.setIZone(kIz);
    pidControllerBR1.setFF(kFF);
    pidControllerBR1.setOutputRange(kMinOutput, kMaxOutput);

    pidControllerBR2.setP(kP);
    pidControllerBR2.setI(kI);
    pidControllerBR2.setD(kD);
    pidControllerBR2.setIZone(kIz);
    pidControllerBR2.setFF(kFF);
    pidControllerBR2.setOutputRange(kMinOutput, kMaxOutput);
    // set PID coefficients

  }

  public void setModuleAngle(double setPoint) {

    pidControllerBR1.setReference(setPoint, ControlType.kPosition);

  }
  
  public void setModuleSpeed(double setPoint) {

    BackRightNeo2.set(setPoint);


  }

  public double getModuleAngle() {
    double first = BackRightNeo1.getEncoder().getPosition();
    return first*60;
  }

  public double getWheelAngle() {
    double second = BackRightNeo2.getEncoder().getPosition();
    return second;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Driving());
  }
}