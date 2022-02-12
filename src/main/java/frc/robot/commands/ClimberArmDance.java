// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberArm;

public class ClimberArmDance extends CommandBase {
  private final ClimberArm _climberArm;
  private final Joystick _joystick; 
  
  /** Creates a new ClimberArmDance. */
  public ClimberArmDance(ClimberArm climberArm, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    _climberArm = climberArm;
    _joystick = joystick;

    addRequirements(climberArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _climberArm.go(_joystick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
