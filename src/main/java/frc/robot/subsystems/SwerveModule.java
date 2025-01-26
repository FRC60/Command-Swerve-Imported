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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;


import com.studica.frc.AHRS;

//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.OperatorConstants;

public class SwerveModule extends SubsystemBase {
  // Stuff here
  private SparkMax driveMotor;
  private SparkMaxConfig configDriveMotor;
  private RelativeEncoder relativeDriveEncoder;
  private SparkClosedLoopController driveMotorPIDController;

  //private EncoderConfig encoderConfigRelativeDrive;

  private SparkMax twistMotor;
  private SparkMaxConfig configTwistMotor;
  private RelativeEncoder relativeTwistEncoder;
  private SparkClosedLoopController twistMotorPIDCOntroller;

  private DutyCycleEncoder absoluteTwistEncoder;

  private double wheelCirc = (3.58 * 0.0254) * Math.PI;
  private double wheelCircMeters = 4.5 *0.0254;

  private double RPMToPID = 0.84;

  private String name;

  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorCANID, int twistMotorCANID, int absoluteTwistEncoderPort, double absoluteEncoderOffset, boolean driveInvert, boolean twistInvert, String moduleName) {
    name = moduleName;

    //Drive Motor
    driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);

    configDriveMotor = new SparkMaxConfig();
    configDriveMotor
      .inverted(driveInvert)
      .smartCurrentLimit(OperatorConstants.AMPLimitDrive);
    configDriveMotor.encoder
      .positionConversionFactor(OperatorConstants.driveMotorFactor)
      .velocityConversionFactor(OperatorConstants.driveMotorVelocityFactor);
   
    configDriveMotor.closedLoop
      .pidf(OperatorConstants.driveMotorP, OperatorConstants.driveMotorI, OperatorConstants.driveMotorD, OperatorConstants.driveMotorFF)
      .maxMotion
      .maxAcceleration(5000)
      .allowedClosedLoopError(500);

    //driveMotor.configure(configDriveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.driveMotorPIDController = driveMotor.getClosedLoopController();
    //driveMotorPIDCOntroller.setSmartMotionAllowedClosedLoopError(absoluteEncoderOffset);

    // relative drive encoder
    this.relativeDriveEncoder = driveMotor.getEncoder(); 



    // Twist Motor
    twistMotor = new SparkMax(twistMotorCANID, MotorType.kBrushless);
    configTwistMotor = new SparkMaxConfig();  

    configTwistMotor
      .inverted(twistInvert)
      .smartCurrentLimit(OperatorConstants.AMPLimitSteering);
    configTwistMotor.encoder
      .positionConversionFactor(OperatorConstants.SteeringMotorFactor);
    configTwistMotor.closedLoop
      .pidf(OperatorConstants.twistMotorP, OperatorConstants.twistMotorI, OperatorConstants.twistMotorD, OperatorConstants.twistMotorFF)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(-180, 180);

    //twistMotor.configure(configDriveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //TODO call the configure thing above^
    
    
    // relative twist encoder
    this.relativeTwistEncoder = twistMotor.getEncoder();
    this.twistMotorPIDCOntroller = twistMotor.getClosedLoopController();

    // absolute twist encoder
    this.absoluteTwistEncoder = new DutyCycleEncoder(absoluteTwistEncoderPort);
    //absoluteTwistEncoder.setDistancePerRotation(absoluteTwistEncoderPort);
    //absoluteTwistEncoder.setPositionOffset(absoluteTwistEncoderPort);

    // Sets twist encoder to correct angle based on absolute zero
    zeroEncoder(absoluteEncoderOffset);








  }

  // RPM to Meters per Second
  double RPMToMetersPerSecond(double RPM){
    return wheelCircMeters * RPM;
  }

  // Subtracts two angles
  public double angleSubtractor (double firstAngle, double secondAngle) {
    // 
    double result = ((firstAngle - secondAngle) + 360180)%360 - 180;
    return result;
  }

  double speedToRPM(double speed){
    if(Math.abs(speed) < 0.05){
      return 0;
    } else{
      return speed * 5000;
    }
    
  }

  public void drive (double speed, double angle){
    //angleEncoder.getVelocity()
    double desiredModRPM = speedToRPM(speed);
    double velocity = relativeDriveEncoder.getVelocity();
    SmartDashboard.putNumber("actual RPM", velocity);
    if(speed!= 0){
 
      /* System.out.println(speed + "speed");
      SmartDashboard.putNumber("speed", speed);
      System.out.println(desiredModRPM + "DesiredRPM");
      SmartDashboard.putNumber("desiredModRPM", desiredModRPM);
      //System.out.println(RPMToMetersPerSecond(desiredModRPM)+" m/sec");
      System.out.println(velocity);

      SmartDashboard.putNumber("Error", desiredModRPM-velocity);
      System.out.println(""); */
    }
    
    double currentAngle = relativeTwistEncoder.getPosition();
    /*** if robot turns the wrong direction, then this may need to be inversed */
    //speed = speed * 0.5
    double setPointAngle = angleSubtractor(currentAngle, angle);
    double setPointAngleFlipped = angleSubtractor(currentAngle + 180, angle);
    //twistMotorPIDCOntroller.setReference(angle, CANSparkMax.ControlType.kPosition);
    //speedMotor.set(speed);
    //System.out.println(desiredModRPM + "desiredModRPM");
    if (Math.abs(setPointAngle) < Math.abs(setPointAngleFlipped)){
      if (Math.abs(speed) < 0.1){
        //twistMotor.set(1);
          System.out.println("Twist" + currentAngle);
          twistMotorPIDCOntroller.setReference(currentAngle, SparkMax.ControlType.kPosition);
          /* System.out.println(name + currentAngle + "_"+ twistMotor.getOutputCurrent());
          System.out.println(twistMotor.getOutputCurrent());
          System.out.println("speed small"); */
          //driveMotor.set(speed);
          System.out.println(name + desiredModRPM/RPMToPID);
          //driveMotorPIDController.setReference(desiredModRPM/RPMToPID, SparkBase.ControlType.kVelocity);
          driveMotorPIDController.setReference(desiredModRPM/RPMToPID, SparkBase.ControlType.kMAXMotionVelocityControl);
          
          
          
      } else {
        //twistMotor.set(1);
          System.out.println("Twist" + angleSubtractor(currentAngle, setPointAngle));
          twistMotorPIDCOntroller.setReference(angleSubtractor(currentAngle, setPointAngle), SparkMax.ControlType.kPosition);
          /* System.out.println(name + angleSubtractor(currentAngle, setPointAngle) + " "+ twistMotor.getOutputCurrent());
          System.out.println(twistMotor.getOutputCurrent()); */
          //driveMotor.set(speed);
          System.out.println(name + desiredModRPM/RPMToPID);
          driveMotorPIDController.setReference(desiredModRPM/RPMToPID, SparkBase.ControlType.kVelocity);
      }
    } else {
      //System.out.println("flipped angle is smaller or equal");
      if (Math.abs(speed) < 0.1){
        //twistMotor.set(1);
        System.out.println("Twist" + currentAngle);
          twistMotorPIDCOntroller.setReference(currentAngle, SparkMax.ControlType.kPosition);
          /* System.out.println(name + currentAngle + " "+ twistMotor.getOutputCurrent());
          System.out.println(twistMotor.getOutputCurrent());
          System.out.println("speed small"); */
          //driveMotor.set(-1 * speed);
          System.out.println(name + -desiredModRPM/RPMToPID);
          driveMotorPIDController.setReference(-desiredModRPM/RPMToPID, SparkBase.ControlType.kVelocity);
      } else {
        //twistMotor.set(1);
        System.out.println("Twist" + angleSubtractor(currentAngle, setPointAngleFlipped));
          twistMotorPIDCOntroller.setReference(angleSubtractor(currentAngle, setPointAngleFlipped), SparkMax.ControlType.kPosition);
  /*         System.out.println(name + angleSubtractor(currentAngle, setPointAngleFlipped) + " " +twistMotor.getOutputCurrent());
          System.out.println(twistMotor.getOutputCurrent()); */
          //driveMotor.set(-1 * speed);
          System.out.println(name + -desiredModRPM/RPMToPID);
          driveMotorPIDController.setReference(-desiredModRPM/RPMToPID, SparkBase.ControlType.kVelocity);
      }
    } 

 }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
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

  private void configAngleMotor(){
    
  }

  //TODO call this at beginning of match too
  void zeroEncoder(double offset){
    //relativeTwistEncoder.setPosition(absoluteTwistEncoder.getAbsolutePosition()* 360 - offset);  
    //TODO we probally need to change the absolute encoder settings
    relativeTwistEncoder.setPosition(absoluteTwistEncoder.get() * 360 - offset); 
    
  }   



  double returnModuleSpeed(){
    // TODO convert module speed to meters per second
    double moduleSpeed = relativeDriveEncoder.getVelocity();
    moduleSpeed = 0;
    return moduleSpeed;
  }

  double returnAbsEncoder(){
    return absoluteTwistEncoder.get();
  }

  double returnTwistEncoder(){
    return relativeTwistEncoder.getPosition();
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
