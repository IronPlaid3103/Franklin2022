// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.Settings;

public class Climber extends SubsystemBase {
  private final CANSparkMax _climberMotor = new CANSparkMax(ClimberConstants.climberMotor, MotorType.kBrushless);
  private final Solenoid _climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.climberSolenoid);
  private double motorPower = Constants.ClimberConstants.climberMotorPower;

  /** Creates a new Climber. */
  public Climber() {}

  public void _climberMotorUp() {
    _climberMotor.set(motorPower);
  }

  public void _climberMotorDown() {
    _climberMotor.set(-motorPower);
  }

  public void _climberSolenoidBack() {
    _climberSolenoid.set(true);
  }

  public void _climberSolenoidForward() {
    _climberSolenoid.set(false); 
  }

  public void setMotorPower(double power1) {
    motorPower = power1;
  }

  public double getMotorPower() {
    return motorPower;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorPower = Settings.getLiveDouble("Climber", "MotorPower", Constants.ClimberConstants.climberMotorPower);
  }
}
