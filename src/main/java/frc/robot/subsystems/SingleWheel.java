// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;



import edu.wpi.first.wpilibj.DutyCycleEncoder;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.OperatorConstants;

public class SingleWheel extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig configMotor;
  private SparkClosedLoopController singleMotorPID;
  private RelativeEncoder encoder;


  /** Creates a new ExampleSubsystem. */
  public SingleWheel() {
    this.motor = new SparkMax(9, MotorType.kBrushless);
    configMotor = new SparkMaxConfig();

    configMotor
      .smartCurrentLimit(OperatorConstants.AMPLimitDrive);
    configMotor.encoder
      .velocityConversionFactor(1);
    configMotor.closedLoop
      .pidf(0.001, 0, 0, 0);
    
    this.encoder = motor.getEncoder();
    this.singleMotorPID = motor.getClosedLoopController();

    motor.configure(configMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
          singleMotorPID.setReference(300/0.84, SparkBase.ControlType.kVelocity);
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
