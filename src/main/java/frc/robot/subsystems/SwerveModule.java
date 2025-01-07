// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
  private CANSparkMax driveMotor;
  private CANSparkMax twistMotor;
  private RelativeEncoder relativeTwistEncoder;
  private DutyCycleEncoder absoluteTwistEncoder;
  private RelativeEncoder relativeDriveEncoder;
  private SparkPIDController twistMotorPIDCOntroller;
  private SparkPIDController driveMotorPIDCOntroller;

  private double wheelCirc = (3.58 * 0.0254) * Math.PI;
  private double wheelCircMeters = 4.5 *0.0254;

  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorCANID, int twistMotorCANID, int absoluteTwistEncoderPort, double absoluteEncoderOffset, boolean driveInvert, boolean twistInvert) {
    //Drive Motor
    this.driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
    this.driveMotor.setInverted(driveInvert);
    this.driveMotor.setSmartCurrentLimit(OperatorConstants.AMPLimitDrive);
 
    //driveMotor.setClosedLoopRampRate(twistMotorCANID);
    //driveMotor.setIdleMode(null);
   
    // Twist Motor
    this.twistMotor = new CANSparkMax(twistMotorCANID, MotorType.kBrushless);
    twistMotor.setInverted(twistInvert);
    twistMotor.setSmartCurrentLimit(OperatorConstants.AMPLimitSteering);

    // relative twist encoder
    this.relativeTwistEncoder = twistMotor.getEncoder();
    relativeTwistEncoder.setPositionConversionFactor(OperatorConstants.SteeringMotorFactor);
    
    // absolute twist encoder
    this.absoluteTwistEncoder = new DutyCycleEncoder(absoluteTwistEncoderPort);
    //absoluteTwistEncoder.setDistancePerRotation(absoluteTwistEncoderPort);
    //absoluteTwistEncoder.setPositionOffset(absoluteTwistEncoderPort);

    // Sets twist encoder to correct angle based on absolute zero
    zeroEncoder(absoluteEncoderOffset);

    // relative drive encoder
    this.relativeDriveEncoder = driveMotor.getEncoder(); 
    relativeDriveEncoder.setPositionConversionFactor(OperatorConstants.driveMotorFactor);
    relativeDriveEncoder.setVelocityConversionFactor(OperatorConstants.driveMotorVelocityFactor);
    //relativeDriveEncoder.setMeasurementPeriod(42);
    //relativeDriveEncoder.setAverageDepth();

    this.twistMotorPIDCOntroller = twistMotor.getPIDController();
    this.driveMotorPIDCOntroller = driveMotor.getPIDController();
    //driveMotorPIDCOntroller.setSmartMotionAllowedClosedLoopError(absoluteEncoderOffset);
    configPID();
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
    return speed * 5000;
  }

  public void drive (double speed, double angle){
    //angleEncoder.getVelocity()
    double desiredModRPM = speedToRPM(speed);
    if(speed!= 0){
      double velocity = relativeDriveEncoder.getVelocity();
      System.out.println(speed + "speed");
      SmartDashboard.putNumber("speed", speed);
      System.out.println(desiredModRPM + "DesiredRPM");
      SmartDashboard.putNumber("desiredModRPM", desiredModRPM);
      //System.out.println(RPMToMetersPerSecond(desiredModRPM)+" m/sec");
      System.out.println(velocity);
      SmartDashboard.putNumber("actual RPM", velocity);
      SmartDashboard.putNumber("Error", desiredModRPM-velocity);
      System.out.println("");
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
          twistMotorPIDCOntroller.setReference(currentAngle, CANSparkMax.ControlType.kPosition);
          //driveMotor.set(speed);
          driveMotorPIDCOntroller.setReference(desiredModRPM, CANSparkBase.ControlType.kVelocity);
      } else {
          twistMotorPIDCOntroller.setReference(angleSubtractor(currentAngle, setPointAngle), CANSparkMax.ControlType.kPosition);
          //driveMotor.set(speed);
          driveMotorPIDCOntroller.setReference(desiredModRPM, CANSparkBase.ControlType.kVelocity);
      }
    } else {
      //System.out.println("flipped angle is smaller or equal");
      if (Math.abs(speed) < 0.1){
          twistMotorPIDCOntroller.setReference(currentAngle, CANSparkMax.ControlType.kPosition);
          //driveMotor.set(-1 * speed);
          driveMotorPIDCOntroller.setReference(-desiredModRPM, CANSparkBase.ControlType.kVelocity);
      } else {
          twistMotorPIDCOntroller.setReference(angleSubtractor(currentAngle, setPointAngleFlipped), CANSparkMax.ControlType.kPosition);
          //driveMotor.set(-1 * speed);
          driveMotorPIDCOntroller.setReference(-desiredModRPM, CANSparkBase.ControlType.kVelocity);
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
    relativeTwistEncoder.setPosition(absoluteTwistEncoder.getAbsolutePosition()* 360 - offset);  
  }   

  private void configPID(){
    driveMotorPIDCOntroller.setP(OperatorConstants.driveMotorP);
    driveMotorPIDCOntroller.setI(OperatorConstants.driveMotorI);
    driveMotorPIDCOntroller.setD(OperatorConstants.driveMotorD);
    driveMotorPIDCOntroller.setFF(OperatorConstants.driveMotorFF);
 
    twistMotorPIDCOntroller.setP(OperatorConstants.twistMotorP);
    twistMotorPIDCOntroller.setI(OperatorConstants.twistMotorI);
    twistMotorPIDCOntroller.setD(OperatorConstants.twistMotorD);
    twistMotorPIDCOntroller.setFF(OperatorConstants.twistMotorFF);
    twistMotorPIDCOntroller.setPositionPIDWrappingEnabled(true);
    twistMotorPIDCOntroller.setPositionPIDWrappingMaxInput(180);
    twistMotorPIDCOntroller.setPositionPIDWrappingMinInput(-180);
  }

  double returnModuleSpeed(){
    // TODO convert module speed to meters per second
    double moduleSpeed = relativeDriveEncoder.getVelocity();
    moduleSpeed = 0;
    return moduleSpeed;
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
