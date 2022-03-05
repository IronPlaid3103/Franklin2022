// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberArm;

public class ClimberArmDownToPosition extends CommandBase {
  private final ClimberArm _climberArm;
  private final double _position;
  
  /** Creates a new ClimberArmDownToPosition. */
  public ClimberArmDownToPosition(ClimberArm climber, double position) {
    _climberArm = climber;
    _position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_climberArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _climberArm.down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _climberArm.isAtPosition(_position);
  }
}
