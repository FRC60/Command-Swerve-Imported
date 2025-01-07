// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.OperatorConstants;

public class SingleWheel extends SubsystemBase {
  private CANSparkMax motor;
  private SparkPIDController singleMotorPID;
  private RelativeEncoder encoder;


  /** Creates a new ExampleSubsystem. */
  public SingleWheel() {
    this.motor = new CANSparkMax(9, MotorType.kBrushless);
    this.motor.setSmartCurrentLimit(OperatorConstants.AMPLimitDrive);
    this.encoder = motor.getEncoder();
    this.singleMotorPID = motor.getPIDController();
    configPID();
  }

  private void configPID(){
    singleMotorPID.setP(0.001);
    singleMotorPID.setI(0);
    singleMotorPID.setD(0);
    singleMotorPID.setFF(0);
    //encoder.setPositionConversionFactor(0.);
    encoder.setVelocityConversionFactor(1);
    
 

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand(DoubleSupplier joystickForward) {
    double value = joystickForward.getAsDouble();
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          SmartDashboard.putNumber("velocity", encoder.getVelocity());
          System.out.println(encoder.getVelocity() + "velocity");
          singleMotorPID.setReference(300/0.84, CANSparkBase.ControlType.kVelocity);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
