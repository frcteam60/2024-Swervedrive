// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.nio.ShortBuffer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class ShooterAndIntake {
    CANSparkMax Rshooter = new CANSparkMax(13, MotorType.kBrushless);
    CANSparkMax Lshooter = new CANSparkMax(14, MotorType.kBrushless);

    CANSparkMax shooterAngle = new CANSparkMax(15, MotorType.kBrushless);

    CANSparkMax intakeLow = new CANSparkMax(16, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax intakeHigh = new CANSparkMax(17, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    SparkMaxPIDController angle_PidController;
    // SwerveDrive constructor
    public ShooterAndIntake (boolean invertRShooter, boolean invertLShooter, boolean invertShooterAngle, boolean invertIntakeLow, boolean invertIntakeHigh){

        // Shooter
        Rshooter.setInverted(invertRShooter);
        Lshooter.setInverted(invertLShooter);
        shooterAngle.setInverted(invertShooterAngle);

        intakeLow.setInverted(invertIntakeLow);
        intakeHigh.setInverted(invertIntakeHigh);
        
    }

    void shoot(double speed, double angle){
        Rshooter.set(speed);
        Lshooter.set(speed);

        shooterAngle.set(angle)       
    }

    void Shooter(){

    }
    void invertDriveMotor(boolean isInverted){
        speedMotor.setInverted(isInverted);
    }
    void invertAngleMotor(boolean isInverted){
        angleMotor.setInverted(isInverted);
    }

    void InvertClimber(boolean dMotorInvert, boolean aMotorInvert){
        shooter.setInverted(dMotorInvert);
        angleMotor.setInverted(aMotorInvert);
    }
}
