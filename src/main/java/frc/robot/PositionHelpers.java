package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionHelpers {
    private static boolean allianceIsBlue = false;

    private static SwerveDrive swerveDriveRef;

    public static void setAllianceIsBlue(boolean allianceIsBlue) {
        PositionHelpers.allianceIsBlue = allianceIsBlue;
    }

    public static Pose2d getSpeakerPosition() {
        double speakerX = 0.24;
        double speakerY = allianceIsBlue ? 5.548 : 2.656;
        return new Pose2d(speakerX, speakerY, new Rotation2d());
    }

    public static double getSpeakerDistance() {
        Pose2d speakerPos = getSpeakerPosition();
        return Math.hypot(swerveDriveRef.returnX() - speakerPos.getX(), swerveDriveRef.returnY() - speakerPos.getY());
    }

    // assigns the reference to the swerve, needed to get robot positions
    public static void assignSwerve(SwerveDrive swerveDriveReference) {
        swerveDriveRef = swerveDriveReference;
    }
}
