package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

public class Shooter extends SubsystemBase 
{

  private double flywheelTolerance = 0.05; // Tolerance of PID controller
  private boolean override = false; // Helps us switch from manual to auto
  private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts
  private double overrideTime = 1.0;

  private final WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.LeftFlywheelPort);
  private final WPI_TalonSRX rightFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.RightFlywheelPort);

  private final PIDController leftFlywheelPID = new PIDController(Constants.leftFlywheelPIDConstants.pidP, Constants.leftFlywheelPIDConstants.pidI, Constants.leftFlywheelPIDConstants.pidD);
  private final PIDController rightFlywheelPID = new PIDController(Constants.rightFlywheelPIDConstants.pidP, Constants.rightFlywheelPIDConstants.pidI, Constants.rightFlywheelPIDConstants.pidD);

  private SimpleMotorFeedforward leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  private SimpleMotorFeedforward rightFlywheelFF = new SimpleMotorFeedforward(Constants.rightFlywheelFF.kS, Constants.rightFlywheelFF.kV, Constants.rightFlywheelFF.kA);


  public Shooter() 
  {
    leftFlywheel.configFactoryDefault();
    leftFlywheel.setInverted(true); leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightFlywheel.configFactoryDefault();
    rightFlywheel.setInverted(false);    rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    leftFlywheelPID.setTolerance(flywheelTolerance);
    rightFlywheelPID.setTolerance(flywheelTolerance);

    overrideTimer.start(); // Start timer
    overrideTimer.reset(); // Reset timer
  }

  public double getLeftRPM() 
  {
    return ((leftFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getRightRPM() 
  {
    return ((rightFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }
    
  public double getLeftFlywheelPower() 
  {
    return leftFlywheel.get();
  }
    
  public double getRightFlywheelPower() 
  {
    return rightFlywheel.get();
  }

  public void setFlywheelPower(double speed) 
  {
    leftFlywheel.set(speed);
    rightFlywheel.set(speed);
  }

  public boolean flywheelWithinErrorMargin() 
  {
    return (leftFlywheelPID.atSetpoint() && rightFlywheelPID.atSetpoint());
  }
  
  public void setFlywheelConstantVelocity(double RPM) 
  {
    leftFlywheel.setVoltage((leftFlywheelFF.calculate(RPM/60.0)*60.0) + leftFlywheelPID.calculate(getLeftRPM(), RPM));
    rightFlywheel.setVoltage((rightFlywheelFF.calculate(RPM/60.0)*60.0) + rightFlywheelPID.calculate(getRightRPM(), RPM));
  }
    
  public double getAverageRPM() 
  {
    return ((getLeftRPM() + getRightRPM())/2.0);
  }

  public double getFlywheelCurrent() 
  {
    return (leftFlywheel.getStatorCurrent() + rightFlywheel.getStatorCurrent())/2.0;
  }
  
  public void resetFlywheelEncoders() 
  {
    leftFlywheel.setSelectedSensorPosition(0, 0, 10);
    rightFlywheel.setSelectedSensorPosition(0, 0, 10);
  }
    

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Average RPM", getAverageRPM());
    SmartDashboard.putNumber("Average Current", getFlywheelCurrent());
    SmartDashboard.putNumber("Left Flywheel RPM", getLeftRPM());
    SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());
    SmartDashboard.putNumber("Right Flywheel RPM", getRightRPM());
    SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());
    
    if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >= overrideTime) 
    {
      override = !override;
      overrideTimer.reset();
    }

    if (override) 
    { 
      if (RobotContainer.getJoy1().getRawButton(1)) 
      {
        setFlywheelConstantVelocity(1000.0); // Sets it to 1000 RPM
      } 
      else 
      {
        setFlywheelConstantVelocity(0.0);
        setFlywheelPower(0.0);
      }
    } 
    else if (!override) 
    { 
      setFlywheelPower(-1.0*RobotContainer.getJoy1().getY());
    }
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
