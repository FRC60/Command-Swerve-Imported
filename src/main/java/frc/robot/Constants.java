// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Joysticks
    public static final int kDriverControllerPort = 0;

    // drive current limits
    public static final int AMPLimitSteering = 20;
    public static final int AMPLimitDrive = 75;

    // Swerve module motor ID's
    public static final int frontLeftSteeringCANID = 8;
    public static final int frontRightSteeringCANID = 6;
    public static final int backLeftSteeringCANID = 2;
    public static final int backRightSteeringCANID = 4;

    public static final int frontLeftDriveCANID = 7;
    public static final int frontRightDriveCANID = 5;
    public static final int backLeftDriveCANID = 1;
    public static final int backRightDriveCANID = 3;

    // motor inverts
    public static final boolean frontLeftSteeringInvert = true;
    public static final boolean frontRightSteeringInvert = true;
    public static final boolean backLeftSteeringInvert = true;
    public static final boolean backRightSteeringInvert = true;

    public static final boolean frontLeftDriveInvert = true;
    public static final boolean frontRightDriveInvert = false;
    public static final boolean backLeftDriveInvert = true;
    public static final boolean backRightDriveInvert = false;

    //position conversion swerve moduels
    //public static final double driveMotorFactor = 1;
    //public static final double driveMotorVelocityFactor = 1;
    //public static final double SteeringMotorFactor = (7.0/96) * 360;
    public static final double driveMotorFactor = (102/13)/50.7;
    public static final double driveMotorVelocityFactor = (102/13);
    public static final double SteeringMotorFactor = (7.0/96) * 360;

    // Absolute Encoder Ports
    public static final int frontLeftAbsoluteEncoderPort = 3;
    public static final int frontRightAbsoluteEncoderPort = 2;
    public static final int backLeftAbsoluteEncoderPort = 0;
    public static final int backRightAbsoluteEncoderPort = 1;

    // AbsolutEncoderOffset
    public static final double frontLeftAbsoluteEncoderOffset = -0.493 * 360;
    public static final double frontRightAbsoluteEncoderOffset = -0.394 * 360;
    public static final double backLeftAbsoluteEncoderOffset = 0.275 * 360;
    public static final double backRightAbsoluteEncoderOffset = -0.257 * 360;

    // PID constants
    public static final double twistMotorP = 0.005;
    public static final double twistMotorI = 0;
    public static final double twistMotorD = 0;
    public static final double twistMotorFF = 0;

    // PID constants
    //0.0004;
    //public static final double driveMotorP = 0.0003;
    public static final double driveMotorP = 0.0001;
    public static final double driveMotorI = 0;
    public static final double driveMotorD = 0;
    //public static final double driveMotorFF = 0.00009;
    public static final double driveMotorFF = 0.;
    

    // Sensitivity
    public static final double sensitivity = 3;

  }
}
