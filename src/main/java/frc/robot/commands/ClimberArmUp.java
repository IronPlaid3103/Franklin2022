// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberArm;

public class ClimberArmUp extends CommandBase {
  private final ClimberArm _climberArm;
  
  /** Creates a new ClimberMotorUp. */
  public ClimberArmUp(ClimberArm climber) {
    _climberArm = climber;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_climberArm); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _climberArm.up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _climberArm.isUp();
  }
}
