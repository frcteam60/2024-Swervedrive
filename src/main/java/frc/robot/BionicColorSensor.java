package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BionicColorSensor {

    ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);

    BionicColorSensor() {
    }

    int getProximity() {
        return colorSensor.getProximity();
    }

    int getIR() {
        return colorSensor.getIR();
    }

    boolean sensesNote() {
        return colorSensor.getIR() > 48;
        // return colorSensor.getIR() > 49;
    }

    void updateDashboard() {
        SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
        SmartDashboard.putNumber("IR", colorSensor.getIR());
    }

}
