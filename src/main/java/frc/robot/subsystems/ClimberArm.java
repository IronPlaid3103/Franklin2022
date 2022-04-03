// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.util.Settings;

public class ClimberArm extends SubsystemBase {
  private final CANSparkMax _climberMotor = new CANSparkMax(ClimberConstants.climberMotor, MotorType.kBrushless);
  private double _motorPower = ClimberConstants.climberArmPower;
  private RelativeEncoder _encoder = _climberMotor.getEncoder();
  private boolean _softLimitEnabled = false;
  private float _limit = ClimberConstants.REVERSE_LIMIT;

  /** Creates a new ClimberArms. */
  public ClimberArm() {
    _climberMotor.restoreFactoryDefaults();

    _climberMotor.setIdleMode(IdleMode.kBrake);

    toggleSoftLimit(); 

    encoderReset();

    _climberMotor.burnFlash();
  }

  public void toggleSoftLimit() {
    _softLimitEnabled = !_softLimitEnabled;

    _climberMotor.enableSoftLimit(SoftLimitDirection.kForward, _softLimitEnabled);
    _climberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.FORWARD_LIMIT);
    _climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, _softLimitEnabled);
    _climberMotor.setSoftLimit(SoftLimitDirection.kReverse, _limit); //changed this --- cannot convert double to float

    SmartDashboard.putBoolean("Soft Limit", _softLimitEnabled);
  }

  public void encoderReset() {
    _encoder.setPosition(0.0);
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

  public void setLimit(double topLimit) {
    topLimit = SmartDashboard.getNumber("Top", ClimberConstants.REVERSE_LIMIT); 
    _limit = (float)topLimit;
  }

  public float getLimit() {
    return _limit;
  }
  //i made this up

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _motorPower = Settings.getLiveDouble("Arm", "Power", ClimberConstants.climberArmPower);

    SmartDashboard.putBoolean("Arm Up", isUp());
    SmartDashboard.putBoolean("Arm Down", isDown());
    SmartDashboard.putNumber("Arm Position", _encoder.getPosition());

    SmartDashboard.putNumber("Arm Current", _climberMotor.getOutputCurrent());
  }

  public boolean isDown() {
    return (_encoder.getPosition() >= ClimberConstants.FORWARD_LIMIT);
  }

  public boolean isUp() {
    return (_encoder.getPosition() <= _limit); //i changed this
  }

  public boolean isAtPosition(double position) {
    return (Math.abs(_encoder.getPosition() - position) < 5);
  }
}
