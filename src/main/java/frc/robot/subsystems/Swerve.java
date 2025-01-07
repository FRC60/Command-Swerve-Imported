// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;



public class Swerve extends SubsystemBase {
  // dimensions between wheels center-to-center
  public static final double length = 22.25;
  public static final double width = 22.25;

  // Gyro
  double gyro_radians;
  double gyro_degrees;
  double temp;
  double forward;
  double strafe;
  double turning;
  double diagonal;
  // Desired Position
  double desiredX = 0;
  double desiredY = 0;
  double desiredYaw = 0;
  double XError;
  double YError;
  double YawError;
  //
  double newX;
  double newY;
  double newRobotAngle;
  Rotation2d gyroRotation2d;
  Pose2d robotPose2d;
  // ***

  // Subtracts two angles
  public double angleSubtractor(double firstAngle, double secondAngle) {
    double result = (((firstAngle - secondAngle) + 360180) % 360) - 180;
    return result;
  }
  
  // ***
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  double yawOffset = 0;

  // ***
  double getGyroRobotYaw() {
    return angleSubtractor(yawOffset, gyro.getYaw());
  }

  public void zeroGyro() {
    yawOffset = 0;
    gyro.zeroYaw();
  }

  void setYawOffset(double newYawOffset) {
    yawOffset = newYawOffset;
  }
  public void setGyro(){
    setYawOffset(gyro.getYaw());
  }

  double applySensitivity(double orignalValue, double sensitivity){
    //absolute value of value raised to 1/sensitivity, then sign reaplied
    double newValue = Math.abs(orignalValue);
    newValue = Math.pow(newValue, sensitivity/1);
    newValue = Math.copySign(newValue, orignalValue);
    //System.out.println(orignalValue + "orignal value");
    //System.out.println(newValue + "newValue");
    return newValue;
  }

  /** Creates a new ExampleSubsystem. */
  private final SwerveModule frontRightModule = new SwerveModule(
    OperatorConstants.frontRightDriveCANID, OperatorConstants.frontRightSteeringCANID, OperatorConstants.frontRightAbsoluteEncoderPort, OperatorConstants.frontRightAbsoluteEncoderOffset, OperatorConstants.frontRightDriveInvert, OperatorConstants.frontRightSteeringInvert);
  
  private final SwerveModule frontLeftModule = new SwerveModule(
    OperatorConstants.frontLeftDriveCANID, OperatorConstants.frontLeftSteeringCANID, OperatorConstants.frontLeftAbsoluteEncoderPort, OperatorConstants.frontLeftAbsoluteEncoderOffset, OperatorConstants.frontLeftDriveInvert, OperatorConstants.frontLeftSteeringInvert);
  
  private final SwerveModule backRightModule = new SwerveModule(
    OperatorConstants.backRightDriveCANID, OperatorConstants.backRightSteeringCANID, OperatorConstants.backRightAbsoluteEncoderPort, OperatorConstants.backRightAbsoluteEncoderOffset, OperatorConstants.backRightDriveInvert, OperatorConstants.backRightSteeringInvert);
  
  private final SwerveModule backLeftModule = new SwerveModule(
    OperatorConstants.backLeftDriveCANID, OperatorConstants.backLeftSteeringCANID, OperatorConstants.backLeftAbsoluteEncoderPort, OperatorConstants.backLeftAbsoluteEncoderOffset, OperatorConstants.backLeftDriveInvert, OperatorConstants.backLeftSteeringInvert);


  public Swerve() {

  }

  // This function converts inches to meters
  public double inchesToMeters(double inches) {
    return inches * 0.0254;
  }

  // ***
  double degreesToRadians(double degreeAngle) {
    return degreeAngle / 360 * 2 * Math.PI;
  }

  // ***
  double radiansToDegrees(double radianAngle) {
    return radianAngle * 360 / 2 / Math.PI;
  }

  // Coerces a value to range
  public double coerceToRange(double number, double min, double max) {
    double coercedValue;
    if (number >= max) {
      coercedValue = max;
    } else if (number <= min) {
      coercedValue = min;
    } else {
      coercedValue = number;
    }
    return coercedValue;
  }


  // drive method
    // ***
    public void drive(double forwardSpeed, double strafeSpeed, double turningSpeed) {
      double tempHighestSpeed;
  
      diagonal = Math.sqrt((length * length) + (width * width));

      // Convert to chassis speeds
      // ChassisSpeeds chassisSpeeds =
      // kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
      // backLeft.getState(), backRight.getState());

      // ***
      // Adjusts values to field oriented drive
      gyro_degrees = getGyroRobotYaw();
      gyro_radians = degreesToRadians(getGyroRobotYaw());
      temp = forwardSpeed * Math.cos(gyro_radians) + strafeSpeed * Math.sin(gyro_radians);
      strafeSpeed = strafeSpeed * Math.cos(gyro_radians) - forwardSpeed * Math.sin(gyro_radians);
      forwardSpeed = temp;

      /*
       * temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
       * strafe = (forward * -1) * Math.sin(gyro_radians) + strafe *
       * Math.cos(gyro_radians);
       * forward = temp;
       */

      // ***
      double a = strafeSpeed - turningSpeed * (length / diagonal); // back horizontal
      double b = strafeSpeed + turningSpeed * (length / diagonal); // front horizontal
      double c = forwardSpeed + turningSpeed * (width / diagonal); // right vertical
      double d = forwardSpeed - turningSpeed * (width / diagonal); // left vertical

      // Speed Values
      // ***
      double backRightSpeed = Math.hypot(a, c);
      double backLeftSpeed = Math.hypot(a, d);
      double frontRightSpeed = Math.hypot(b, c);
      double frontLeftSpeed = Math.hypot(b, d);
      // Angle Values
      double backRightAngle = (Math.atan2(a, c) / Math.PI * 180);
      double backLeftAngle = (Math.atan2(a, d) / Math.PI * 180);
      double frontRightAngle = (Math.atan2(b, c) / Math.PI * 180);
      double frontLeftAngle = (Math.atan2(b, d) / Math.PI * 180);

      // ***
      // If a speed is more than 100% power scale the speeds down
      /* tempHighestSpeed = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
              Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)));
      if (tempHighestSpeed > 1) {
          frontLeftSpeed = frontLeftSpeed / tempHighestSpeed;
          frontRightSpeed = frontRightSpeed / tempHighestSpeed;
          backLeftSpeed = backLeftSpeed / tempHighestSpeed;
          backRightSpeed = backRightSpeed / tempHighestSpeed;
      } */
      //
      backRightModule.drive(backRightSpeed, backRightAngle);
      backLeftModule.drive(backLeftSpeed, backLeftAngle);
      frontRightModule.drive(frontRightSpeed, frontRightAngle);
      frontLeftModule.drive(frontLeftSpeed, frontLeftAngle);

  }

  public Command driveTeleop(DoubleSupplier joystickForward, DoubleSupplier joystickSideways, DoubleSupplier joystickTurning) {
    return run(
    () -> {
    double turningDT = -joystickTurning.getAsDouble();
    double forwardDT = joystickForward.getAsDouble();
    double sidewaysDT = -joystickSideways.getAsDouble();
    if (turningDT >= -0.03 && turningDT <= 0.03) {
          YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
          if (YawError >= -3 && YawError <= 3) {
              YawError = 0;
          }
          turningDT = coerceToRange((YawError) * 0.010, -1, 1);
          // 0.020
      } else {
          desiredYaw = getGyroRobotYaw() + (turningDT * 10);
          YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
          if (YawError >= -2 && YawError <= 2) {
              YawError = 0;
          }
          turningDT = coerceToRange((YawError) * 0.07, -1, 1);
      }

      if (forwardDT >= -0.01 && forwardDT <= 0.01) {
          forwardDT = 0;
      } else {
          //forwardDT = joystickForward;
      }

      if (sidewaysDT >= -0.01 && sidewaysDT <= 0.01) {
          sidewaysDT = 0;
      } else {
          //sidewaysDT = joystickSideways;
      }
      
      //drive(forwardDT, sidewaysDT, turningDT);
      drive(applySensitivity(forwardDT, OperatorConstants.sensitivity), applySensitivity(sidewaysDT, OperatorConstants.sensitivity), applySensitivity(turningDT, OperatorConstants.sensitivity));
    });

  }


  public void relativeEncoderOffsets(){
    frontLeftModule.zeroEncoder(OperatorConstants.frontLeftAbsoluteEncoderOffset);
    frontRightModule.zeroEncoder(OperatorConstants.frontRightAbsoluteEncoderOffset);
    backLeftModule.zeroEncoder(OperatorConstants.backLeftAbsoluteEncoderOffset);
    backRightModule.zeroEncoder(OperatorConstants.backRightAbsoluteEncoderOffset);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
