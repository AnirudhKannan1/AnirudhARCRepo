// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.AutoPath;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.TurnGyroPID;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

//import frc.robot.subsystems.MaxbotixUltrasonicSensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static DriveTrain _DriveTrain;
  
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;

  private static AutoDriveForward driveForward;

  private final TankDrive _tankDrive;
  private final ArcadeDrive _arcadeDrive;

  private static AutoPath autoPath;
  
  //MaxbotixUltrasonicSensor ultrasonicSensor = new MaxbotixUltrasonicSensor(Constants.I2CAddresses.MaxbotixUltrasonicSensor);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    _leftJoystick = new Joystick(Constants.USBOrder.Two);
    _rightJoystick = new Joystick(Constants.USBOrder.Three);
    _DriveTrain = new DriveTrain();
    _tankDrive = new TankDrive(_DriveTrain, _leftJoystick, _rightJoystick);
    _DriveTrain.setDefaultCommand(_tankDrive);
    _arcadeDrive = new ArcadeDrive(_DriveTrain, _leftJoystick);
    driveForward = new AutoDriveForward(_DriveTrain, -100);
    autoPath = new AutoPath();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoPath;
  }

  public static DriveTrain getDriveTrain(){
    return _DriveTrain;
  }
}
