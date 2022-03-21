// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberHooks extends SubsystemBase {
    private final Solenoid _climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.climberSolenoid);

  /** Creates a new Climber. */
  public ClimberHooks() {}

  public void back() {
    _climberSolenoid.set(true);
  }

  public void forward() {
    _climberSolenoid.set(false); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
