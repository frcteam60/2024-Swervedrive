package frc.robot;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BionicColorSensor {

    ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);

    Queue<Integer> past10Proxs = new LinkedList<>();

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
        // SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
        SmartDashboard.putNumber("IR", colorSensor.getIR());

        past10Proxs.add(colorSensor.getProximity());
        if (past10Proxs.size() > 10) {
            past10Proxs.remove();
        }
        int lowest = past10Proxs.peek();
        int highest = past10Proxs.peek();
        for (Iterator<Integer> itr = past10Proxs.iterator(); itr.hasNext();) {
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
