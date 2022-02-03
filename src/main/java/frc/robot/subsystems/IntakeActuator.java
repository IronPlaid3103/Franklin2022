// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeActuator extends SubsystemBase {
  private final Solenoid _actuator = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.actuatorSolenoid);
  
  /** Creates a new IntakeActuator. */
  public IntakeActuator() {}

  public void actuateIn() {
    _actuator.set(false);
  }

  public void actuateOut() {
    _actuator.set(true);
  }

  public boolean isActuated() {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}