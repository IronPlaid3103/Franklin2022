// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive_Train extends SubsystemBase {
  
  private final CANSparkMax _fLMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax _fRMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax _bLMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax _bRMotor = new CANSparkMax(4, MotorType.kBrushless);

  private final DifferentialDrive _drive = new DifferentialDrive(_fLMotor, _fRMotor); 

  public Drive_Train() {  
    _fLMotor.restoreFactoryDefaults();
    _fRMotor.restoreFactoryDefaults();
    _bLMotor.restoreFactoryDefaults();
    _bRMotor.restoreFactoryDefaults();

    _fLMotor.setInverted(true);
    _bLMotor.setInverted(true);

    _bLMotor.follow(_fLMotor);
    _bRMotor.follow(_fRMotor);
  }

  public void teleopDrive(Joystick driveControl) {
    double forward = driveControl.getRawAxis(1);
    double turn = driveControl.getRawAxis(4); 

    //_drive.curvatureDrive(forward, turn, Math.abs(forward) <= .1);
    _drive.arcadeDrive(forward, -turn);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
