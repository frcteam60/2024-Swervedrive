// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// zero angle encoder

import java.nio.ShortBuffer;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/** Add your docs here. */
public class ShooterAndIntake {
    CANSparkMax Rshooter = new CANSparkMax(9, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax Lshooter = new CANSparkMax(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    CANSparkMax shooterAngle = new CANSparkMax(11, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    CANSparkMax intakeLow = new CANSparkMax(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax intakeHigh = new CANSparkMax(13, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    // DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(0);
    RelativeEncoder angleEncoder;

    double angleLimitUpper;
    double angleLimitLower;
    double lastDirection;

    SparkMaxPIDController angle_PidController;

    BionicColorSensor colorSensor = new BionicColorSensor();
    // TODO set correct speaker height in meters
    double speakerHeight = 0;

    // SwerveDrive constructor
    public ShooterAndIntake(boolean invertRShooter, boolean invertLShooter, boolean invertShooterAngle,
            boolean invertIntakeLow, boolean invertIntakeHigh, double upperLimitAngle, double lowerLimitAngle) {

        double rampRate = 2.3;

        // Shooter
        Rshooter.setInverted(invertRShooter);
        Lshooter.setInverted(invertLShooter);
        Lshooter.setClosedLoopRampRate(rampRate);
        Lshooter.setOpenLoopRampRate(rampRate);
        Rshooter.setClosedLoopRampRate(rampRate);
        Rshooter.setOpenLoopRampRate(rampRate);

        // Intake
        intakeLow.setInverted(invertIntakeLow);
        intakeHigh.setInverted(invertIntakeHigh);
        // Shooter angle
        shooterAngle.setInverted(invertShooterAngle);
        angleEncoder = shooterAngle.getEncoder();
        angleEncoder.setPositionConversionFactor(360.0 / 500);
        shooterAngle.setSoftLimit(SoftLimitDirection.kForward, 122);
        // hardstop is aproximently 21.8
        shooterAngle.setSoftLimit(SoftLimitDirection.kReverse, 25);
        shooterAngle.enableSoftLimit(SoftLimitDirection.kForward, true);
        shooterAngle.enableSoftLimit(SoftLimitDirection.kReverse, true);
        angleLimitUpper = upperLimitAngle;
        angleLimitLower = lowerLimitAngle;

    }

    void shooter(double speed) {
        double intakeSpeed = -0.2;
        if (speed < intakeSpeed) {
            speed = intakeSpeed;
        }
        Rshooter.set(speed);
        Lshooter.set(speed);
    }

    void angle(double direction) {
        shooterAngle.set(direction);
    }

    void setAngle(double desiredAngle) {
        shooterAngle.set((desiredAngle - angleEncoder.getPosition()) * 0.35);
    }

    void shootInSpeaker(boolean isRobotInPlace, double[] poseFromTarget) {
        boolean angleLinedUp = false;
        // Find desired shooter angle
        // shooter angle

        if (angleLinedUp == false) {
            // setAngle();
            // set angleLinedUp
        }
        if (angleLinedUp && isRobotInPlace) {
            // Shoot
        }
    }

    void trapShot(boolean robotLinedUp) {
        shooter(0.275);
        setAngle(58);
        if (Math.abs(angleEncoder.getPosition() - 58) <= 2 && robotLinedUp) {
            intakeHigh.set(1);
        }
    }

    void shootInSubwoofer() {
        if (angleEncoder.getPosition() == 31) {
            shooter(0.15);
        }
    }

    void intake(int direction) {
        if (direction == 0 && colorSensor.sensesNote(returnAngle())) {
            adjustNoteBack(false);
        } else {
            intakeHigh.set(direction * 0.5);
            intakeLow.set(direction * 0.5);
        }
    }

    void intakeSpeed(double speedTop, double speedBottom) {
        intakeHigh.set(speedTop);
        intakeLow.set(speedBottom);
    }

    void shooterToSpeed(double desiredSpeed) {
        if (colorSensor.sensesNote(returnAngle())) {
            // If note is infront of color sensor
            adjustNoteBack(true);
        } else {
            shooter(desiredSpeed);
        }
    }

    void zeroAngleEncoder(double position) {
        angleEncoder.setPosition(position);
    }

    double returnAngle() {
        return angleEncoder.getPosition();
    }

    void setAngleEncoder(double angle) {
        angleEncoder.setPosition(angle);
    }

    void offShooterAndIntake() {
        Rshooter.set(0);
        Lshooter.set(0);
        intakeHigh.set(0);
        intakeLow.set(0);
        shooterAngle.set(0);
    }

    void updateDashboardForColorSensor() {
        colorSensor.updateDashboard();
    }

    private void adjustNoteBack(boolean includeShooter) {
        intakeHigh.set(-0.1);
        intakeLow.set(0);
        if (includeShooter) {
            shooter(-0.05);
        }
    }

    private double getPowerForSpeakerShot() {
        double distance = PositionHelpers.getSpeakerDistance();
        double basePower = .3; // Power used on subwoofer shot, minus some due to distance max distance is ~20
                               // (illegal shot), halfcourt will be around 8.5. This leads to a power of 1.1
                               // and .64, respectively
        return Math.min(1, basePower + 4.0 * distance / 100);
    }

    void setAngleForSpeaker() {
        double distance = PositionHelpers.getSpeakerDistance();
        double power = getPowerForSpeakerShot();
        // TODO convert the distance into an angle for the arm
        setAngle(Math.hypot(distance, speakerHeight));
    }

    void rampForSpeaker() {
        double power = getPowerForSpeakerShot();
        shooterToSpeed(power);
    }
}
