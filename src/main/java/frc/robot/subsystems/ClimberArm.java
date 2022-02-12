// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.util.Settings;

public class ClimberArm extends SubsystemBase {
  private final CANSparkMax _climberMotor = new CANSparkMax(ClimberConstants.climberMotor, MotorType.kBrushless);
  private double _motorPower = ClimberConstants.climberArmPower;
  
  /** Creates a new ClimberArms. */
  public ClimberArm() {
    setDefaultCommand(new RunCommand(this::stop, this));
  }

  public void go(Joystick operatorControl) {
    double speed = applyDeadband(operatorControl.getRawAxis(JoystickConstants.LEFT_STICK_Y));
    _climberMotor.set(speed);
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < JoystickConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(.1, value)) / (1 - JoystickConstants.deadband);
  }
  
  public void stop() {
    _climberMotor.stopMotor();
  }

  public void up() {
    _climberMotor.set(_motorPower);
  }

  public void down() {
    _climberMotor.set(-_motorPower);
  }

  public void setPower(double power) {
    _motorPower = power;
  }

  public double getPower() {
    return _motorPower;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _motorPower = Settings.getLiveDouble("ClimberArm", "Power", ClimberConstants.climberArmPower);
  }
}
