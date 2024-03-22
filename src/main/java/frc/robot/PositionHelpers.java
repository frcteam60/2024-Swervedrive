package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionHelpers {
    private static boolean allianceIsBlue = false;

    public static void setAllianceIsBlue(boolean allianceIsBlue) {
        PositionHelpers.allianceIsBlue = allianceIsBlue;
    }

    public static Pose2d getSpeakerPosition() {
        double speakerX = 0;
        double speakerY = allianceIsBlue ? 5.548 : 2.656;
        return new Pose2d(speakerX, speakerY, new Rotation2d());
    }

    public static double getSpeakerDistance() {
        Pose2d speakerPos = getSpeakerPosition();
        return Math.hypot(speakerPos.getX(), speakerPos.getY());
    }
}
