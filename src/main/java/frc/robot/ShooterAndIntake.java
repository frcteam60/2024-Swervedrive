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
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/** Add your docs here. */
public class ShooterAndIntake {
    CANSparkMax Rshooter = new CANSparkMax(9, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax Lshooter = new CANSparkMax(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    CANSparkMax shooterAngle = new CANSparkMax(11, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    CANSparkMax intakeLow = new CANSparkMax(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax intakeHigh = new CANSparkMax(13, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    SparkMaxPIDController rPIDController = Rshooter.getPIDController();
    SparkMaxPIDController lPIDController = Lshooter.getPIDController();

    // DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(0);
    RelativeEncoder angleEncoder;
    RelativeEncoder rEncoder;
    RelativeEncoder lEncoder;

    double angleLimitUpper;
    double angleLimitLower;
    double lastDirection;

    SparkMaxPIDController angle_PidController;

    BionicColorSensor colorSensor = new BionicColorSensor();
    double speakerHeight = 2.05;

    //double p = 0.00025;
    double p = 0.0001795;
    double i = 0;
    double d = 0;
    //double ff = 0.0001795;
    double ff = 0.00025;

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

        rEncoder = Rshooter.getEncoder();
        lEncoder = Lshooter.getEncoder();
        
        // Intake
        intakeLow.setInverted(invertIntakeLow);
        intakeHigh.setInverted(invertIntakeHigh);
        // Shooter angle
        shooterAngle.setInverted(invertShooterAngle);
        angleEncoder = shooterAngle.getEncoder();
        angleEncoder.setPositionConversionFactor(360.0 / 500);
        //shooterAngle.setSoftLimit(SoftLimitDirection.kForward, 122);
        shooterAngle.setSoftLimit(SoftLimitDirection.kForward,78);// old 120
        // hardstop is aproximently 21.8
        shooterAngle.setSoftLimit(SoftLimitDirection.kReverse, 25);
        shooterAngle.enableSoftLimit(SoftLimitDirection.kForward, true);
        shooterAngle.enableSoftLimit(SoftLimitDirection.kReverse, true);
        angleLimitUpper = upperLimitAngle;
        angleLimitLower = lowerLimitAngle;

        rPIDController.setFF(ff);
        rPIDController.setP(p);
        rPIDController.setI(i);
        rPIDController.setD(d);
        rPIDController.setOutputRange(-1, 1);
        
        
        lPIDController.setFF(ff);
        lPIDController.setP(p);
        lPIDController.setI(i);
        lPIDController.setD(d);
        lPIDController.setOutputRange(-1, 1);
        
    }


    public double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    double returnRShooterSpeed(){
        return rEncoder.getVelocity();
    }
    double returnLShooterSpeed(){
        return lEncoder.getVelocity();
    }
    void shooter(double speed) {
        double intakeSpeed = -0.2;
        if (speed < intakeSpeed) {
            speed = intakeSpeed;
        }
        Rshooter.set(speed);
        Lshooter.set(speed);
    }

    void topShooter(){
        
    }

    // Sets L RPM
    double setLShooterRPM(double rpm){
        lPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        return rpm - returnLShooterSpeed();
    }
    // Sets R RPM
    double setRShooterRPM(double rpm){
        rPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        return rpm - returnRShooterSpeed();
    }
    
    void shooterControlRPM(double rpm){
        setLShooterRPM(rpm);
        setRShooterRPM(rpm);
    }

    // set speed of the shooter angle
    void angle(double direction) {
        shooterAngle.set(direction);
    }

    // move shooter to desired angle
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

    // Returns shooter angle
    double returnAngle() {
        return angleEncoder.getPosition();
    }
    
    // set angle encoder angle
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

    double getPowerForSpeakerShot() {
        double distance = PositionHelpers.getSpeakerDistance();
        double basePower = .3; // Power used on subwoofer shot, minus some due to distance max distance is ~20
                               // (illegal shot), halfcourt will be around 8.5. This leads to a power of 1.1
                               // and .64, respectively
        return Math.min(1, basePower + 4.0 * distance / 100);
    }

    void setAngleForSpeaker() {
        double distance = PositionHelpers.getSpeakerDistance();
        double angle = Math.toDegrees(Math.atan2(speakerHeight - inchesToMeters(8), distance));
        setAngle(angle + 2);
    }

    void rampForSpeaker() {
        double power = getPowerForSpeakerShot();
        shooterToSpeed(power);
    }

    void setPIDI(double integral){
        lPIDController.setI(integral);
        rPIDController.setI(integral);
    }

    void shooterAngleSoftLimit(boolean enable){
        shooterAngle.enableSoftLimit(SoftLimitDirection.kForward, enable);
        shooterAngle.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }
    

    
    
}
