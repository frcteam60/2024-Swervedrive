// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/** Add your docs here. */
public class Climber {
    TalonSRX climber1 = new TalonSRX(14);
    TalonSRX climber2 = new TalonSRX(15);
    TalonSRX climber3 = new TalonSRX(16);
    TalonSRX climber4 = new TalonSRX(17);

    
    // SwerveDrive constructor
    public Climber (boolean invertClimber1, boolean invertClimber2, boolean invertClimber3, boolean invertClimber4){
        climber1.set(TalonSRXControlMode.PercentOutput, 0);
        climber2.set(TalonSRXControlMode.PercentOutput, 0);
        climber3.set(TalonSRXControlMode.PercentOutput, 0);
        climber4.set(TalonSRXControlMode.PercentOutput, 0);

        climber1.configContinuousCurrentLimit(10);
        climber1.configPeakCurrentDuration(1000);
        climber1.configPeakCurrentLimit(15);

        climber2.configContinuousCurrentLimit(10);
        climber2.configPeakCurrentDuration(1000);
        climber2.configPeakCurrentLimit(15);

        climber3.configContinuousCurrentLimit(10);
        climber3.configPeakCurrentDuration(1000);
        climber3.configPeakCurrentLimit(15);

        climber4.configContinuousCurrentLimit(10);
        climber4.configPeakCurrentDuration(1000);
        climber4.configPeakCurrentLimit(15);
        
        //peak 10 for 1 sec cont 5

        /* climber2.follow(climber1);
        climber4.follow(climber3); */

        climber1.setInverted(invertClimber1);
        climber2.setInverted(invertClimber2);
        climber3.setInverted(invertClimber3);
        climber4.setInverted(invertClimber4);
        
    }
    public void climb(double speed){

        climber1.set(TalonSRXControlMode.PercentOutput, speed);
        climber2.set(TalonSRXControlMode.PercentOutput, speed);
        climber3.set(TalonSRXControlMode.PercentOutput, speed);
        climber4.set(TalonSRXControlMode.PercentOutput, speed);
       
    }
    public void rightClimber(double speed){
        climber3.set(TalonSRXControlMode.PercentOutput, speed);
        climber4.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void leftClimber(double speed){
        climber1.set(TalonSRXControlMode.PercentOutput, speed);
        climber2.set(TalonSRXControlMode.PercentOutput, speed);
    }
    void offClimber(){
        climber1.set(TalonSRXControlMode.PercentOutput, 0);
        climber2.set(TalonSRXControlMode.PercentOutput, 0);
        climber3.set(TalonSRXControlMode.PercentOutput, 0);
        climber4.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
