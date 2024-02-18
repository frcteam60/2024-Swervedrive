// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// zero angle encoder

import java.nio.ShortBuffer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class ShooterAndIntake {
    CANSparkMax Rshooter = new CANSparkMax(9, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax Lshooter = new CANSparkMax(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    CANSparkMax shooterAngle = new CANSparkMax(11, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    CANSparkMax intakeLow = new CANSparkMax(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax intakeHigh = new CANSparkMax(13, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(0);
    RelativeEncoder angleEncoder;

    double angleLimitUpper;
    double angleLimitLower;

    SparkMaxPIDController angle_PidController;
    // SwerveDrive constructor
    public ShooterAndIntake (boolean invertRShooter, boolean invertLShooter, boolean invertShooterAngle, boolean invertIntakeLow, boolean invertIntakeHigh, double upperLimitAngle, double lowerLimitAngle){

        // Shooter
        Rshooter.setInverted(invertRShooter);
        Lshooter.setInverted(invertLShooter);
        // Intake
        intakeLow.setInverted(invertIntakeLow);
        intakeHigh.setInverted(invertIntakeHigh);
        // Shooter angle
        shooterAngle.setInverted(invertShooterAngle);
        angleEncoder = shooterAngle.getEncoder();
        //angleEncoder.setPositionConversionFactor(0);
        angleLimitUpper = upperLimitAngle;
        angleLimitLower = lowerLimitAngle;
        
    }

    void shooter(double speed){
        Rshooter.set(speed);
        Lshooter.set(-speed);     
    }

    void changeAngle(double angleChange){
        if (angleEncoder.getPosition() >= angleLimitUpper && Math.signum(angleChange) == 1){
            shooterAngle.set(0);    
        } else if (angleEncoder.getPosition() <= angleLimitLower && Math.signum(angleChange) == -1){
            shooterAngle.set(0);
        } else {
            shooterAngle.set(angleChange);
        }
    }

    void setAngle(double desiredAngle){
        shooterAngle.set((desiredAngle - angleEncoder.getPosition()) * 0.5);
    }
    void shootInSpeaker(boolean isRobotInPlace, double[] poseFromTarget){
        boolean angleLinedUp = false;
        // Find desired shooter angle
        // shooter angle
        if (angleLinedUp == false) {
            //setAngle();
            // set angleLinedUp
        }
        if (angleLinedUp && isRobotInPlace){
            // Shoot
        }
    }

    void shootInAmp(double[] poseFromTarget){
        //line up
        // if both parts lined up
        // then shoot
        //setAngle(0);
        //shooter(0);
    }
    void intake(int direction){
        intakeHigh.set(direction * 0.5);
        intakeLow.set(direction * 0.5);
    }

    void zeroAngleEncoder(double absoluteOffSet){
        angleEncoder.setPosition(absAngleEncoder.getAbsolutePosition() * 360 - absoluteOffSet);
    }
}
