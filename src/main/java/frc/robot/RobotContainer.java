// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SingleWheel;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Swerve swerveDrive = new Swerve();
  private final SingleWheel motor = new SingleWheel();
  
  // xbox controller
  private final CommandXboxController xboxController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
        
    new Trigger(() -> DriverStation.isDisabled())
        .onTrue(swerveDrive.setDesiredYawCommand());
    
    //m_driverController.y().onTrue(swerveDrive.setDesiredYawCommand(0));
    //m_driverController.y().onTrue(m_intake.retractCommand());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    /*driveTrain.setDefaultCommand(
      driveTrain.simpleDriveCommand(
        () -> joystick.getRawAxis(1)*0.5, () -> joystick.getRawAxis(2)* 0.5)
    );*/
    System.out.println("configurebindings");
    swerveDrive.setDefaultCommand(
      swerveDrive.driveTeleop(() -> -xboxController.getLeftY(), () -> xboxController.getLeftX(), () -> xboxController.getRightX()));
    
    /*motor.setDefaultCommand(
      motor.exampleMethodCommand(() -> -xboxController.getRightTriggerAxis()));
    */
  }

  public void setEncoderOffsets(){
    swerveDrive.relativeEncoderOffsets();
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
  public void setGyro(){
    swerveDrive.setGyro();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
