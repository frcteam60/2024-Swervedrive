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
        climber3.set(TalonSRXControlMode.PercentOutput, 0);

        climber2.follow(climber1);
        climber4.follow(climber3);

        climber1.setInverted(invertClimber1);
        climber2.setInverted(invertClimber2);
        climber3.setInverted(invertClimber3);
        climber4.setInverted(invertClimber4);


    }
    public void climb(double direction){

        climber1.set(TalonSRXControlMode.PercentOutput, direction);
        climber3.set(TalonSRXControlMode.PercentOutput, direction);
       
    }

}
