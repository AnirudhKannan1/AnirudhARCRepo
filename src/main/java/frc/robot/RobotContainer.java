package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;


public class RobotContainer {
  
  private static Joystick _leftJoystick;
  
  private final Shooter shooter = new Shooter();


  public RobotContainer() 
  {
    _leftJoystick = new Joystick(0);
    configureButtonBindings();
  }

  private void configureButtonBindings() 
  {

  }

  public static Joystick getJoy1()
  {
    return _leftJoystick;
  }
}
