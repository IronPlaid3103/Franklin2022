// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonIntake extends SequentialCommandGroup {
  /** Creates a new AutonIntake. */
  public AutonIntake(Intake intake, IntakeActuator intakeActuator, Drive_Train drive_Train, String path, ClimberHooks climberHooks) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeActuateOut(intakeActuator), 
      new ParallelDeadlineGroup(
        new AutonDrivePathWithWait(drive_Train, path, climberHooks), 
        new IntakeIn(intake)
      )
    );
  }
}
