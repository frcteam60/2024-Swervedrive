package frc.robot;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BionicColorSensor {

    ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);

    Queue<Integer> pastXProxs = new LinkedList<>();

    BionicColorSensor() {
    }

    int getProximity() {
        return colorSensor.getProximity();
    }

    int getIR() {
        return colorSensor.getIR();
    }

    boolean sensesNote(double shooterAngle) {
        if (shooterAngle <= 50){
            return colorSensor.getIR() > 48;
        } else if(shooterAngle >= 51 && shooterAngle <= 75){
            return colorSensor.getIR() > 48;
        } else if(shooterAngle >= 76 && shooterAngle <= 100){
            return colorSensor.getIR() > 48;
        } else {
            // if angle is 100-122
            return colorSensor.getIR() > 48;
        }
        // return colorSensor.getIR() > 49;
    }

    void updateDashboard() {
        // SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
        SmartDashboard.putNumber("IR", colorSensor.getIR());

        pastXProxs.add(colorSensor.getProximity());
        if (pastXProxs.size() > 25) {
            pastXProxs.remove();
        }
        int lowest = pastXProxs.peek();
        int highest = pastXProxs.peek();
        for (Iterator<Integer> itr = pastXProxs.iterator(); itr.hasNext();) {
            int val = itr.next();
            if (val < lowest)
                lowest = val;
            if (val > highest)
                highest = val;
        }
        SmartDashboard.putNumber("Proximity low", lowest);
        SmartDashboard.putNumber("Proximity high", highest);
    }

}
