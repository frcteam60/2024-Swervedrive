// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//***
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */
public class SwerveDrive {
    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;
    // dimensions between wheels center-to-center
    public static final double length = 22.25;
    public static final double width = 22.25;

    // Kinematics
    private static Translation2d translation2dfrontRight = new Translation2d(width / 2, -(length / 2));
    private static Translation2d translation2dfrontLeft = new Translation2d(width / 2, (length / 2));
    private static Translation2d translation2dbackRight = new Translation2d(-(width / 2), -(length / 2));
    private static Translation2d translation2dbackLeft = new Translation2d(-(width / 2), length / 2);

    // private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0,
    // Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
    // ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0);
    // private final SwerveDriveOdometry odometry;

    // Gyro
    double gyro_radians;
    double gyro_degrees;

    double temp;
    double forward;
    double strafe;
    double turning;
    double diagonal;

    // Desired Position
    double desiredX = 0;
    double desiredY = 0;
    double desiredYaw = 0;
    double XError;
    double YError;
    double YawError;

    //
    double newX;
    double newY;
    double newRobotAngle;

    Rotation2d gyroRotation2d;
    Pose2d robotPose2d;

    // ***

    // ***
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    double yawOffset = 0;

    // ***
    double getGyroRobotYaw() {
        return angleSubtractor(yawOffset, gyro.getYaw());
    }

    void zeroGyro() {
        yawOffset = 0;
        gyro.zeroYaw();
    }

    void setYawOffset(double newYawOffset) {
        yawOffset = newYawOffset;
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(translation2dfrontLeft,
            translation2dfrontRight, translation2dbackLeft, translation2dbackRight);

    private final SwerveDriveOdometry odometry;
    // private final SwerveDriveOdometry odometry = new
    // SwerveDriveOdometry(kinematics, desiredYaw, backRight.getPosition(),
    // backLeft.getPosition(), frontRight.getPosition(), frontLeft.getPosition());

    // SwerveDrive constructor
    public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        // This is the position of the swerve modules on our robot

        // System.out.println("constructing swerve");

        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;

        gyroRotation2d = new Rotation2d(0);
        robotPose2d = new Pose2d(0, 0, gyroRotation2d);

        this.odometry = new SwerveDriveOdometry(kinematics, gyroRotation2d, new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() },
                robotPose2d);

        // new SwerveDriveOdometry(kinematics, desiredYaw, new SwerveModulePosition[]
        // {backRight.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
        // frontLeft.getPosition()});

    }

    // This function converts inches to meters
    public double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    // Subtracts two angles
    public double angleSubtractor(double firstAngle, double secondAngle) {
        double result = (((firstAngle - secondAngle) + 360180) % 360) - 180;
        return result;

    }

    // ***
    double degreesToRadians(double degreeAngle) {
        return degreeAngle / 360 * 2 * Math.PI;
    }

    // ***
    double radiansToDegrees(double radianAngle) {
        return radianAngle * 360 / 2 / Math.PI;
    }

    // Coerces a value to range
    public double coerceToRange(double number, double min, double max) {
        double coercedValue;
        if (number >= max) {
            coercedValue = max;
        } else if (number <= min) {
            coercedValue = min;
        } else {
            coercedValue = number;
        }

        return coercedValue;
    }

    // drive method
    // ***
    public void drive(double forwardSpeed, double strafeSpeed, double turningSpeed) {
        double tempHighestSpeed;
        diagonal = Math.sqrt((length * length) + (width * width));

        // Convert to chassis speeds
        // ChassisSpeeds chassisSpeeds =
        // kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
        // backLeft.getState(), backRight.getState());

        // ***
        // Adjusts values to field oriented drive
        gyro_degrees = getGyroRobotYaw();
        gyro_radians = degreesToRadians(getGyroRobotYaw());
        temp = forwardSpeed * Math.cos(gyro_radians) + strafeSpeed * Math.sin(gyro_radians);
        strafeSpeed = strafeSpeed * Math.cos(gyro_radians) - forwardSpeed * Math.sin(gyro_radians);
        forwardSpeed = temp;

        /*
         * temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
         * strafe = (forward * -1) * Math.sin(gyro_radians) + strafe *
         * Math.cos(gyro_radians);
         * forward = temp;
         */

        // ***
        double a = strafeSpeed - turningSpeed * (length / diagonal); // back horizontal
        double b = strafeSpeed + turningSpeed * (length / diagonal); // front horizontal
        double c = forwardSpeed + turningSpeed * (width / diagonal); // right vertical
        double d = forwardSpeed - turningSpeed * (width / diagonal); // left vertical

        // Speed Values
        // ***
        double backRightSpeed = Math.hypot(a, c);
        double backLeftSpeed = Math.hypot(a, d);
        double frontRightSpeed = Math.hypot(b, c);
        double frontLeftSpeed = Math.hypot(b, d);
        // Angle Values
        double backRightAngle = (Math.atan2(a, c) / Math.PI * 180);
        double backLeftAngle = (Math.atan2(a, d) / Math.PI * 180);
        double frontRightAngle = (Math.atan2(b, c) / Math.PI * 180);
        double frontLeftAngle = (Math.atan2(b, d) / Math.PI * 180);

        // ***
        // If a speed is more than 100% power scale the speeds down
        /* tempHighestSpeed = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
                Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)));
        if (tempHighestSpeed > 1) {
            frontLeftSpeed = frontLeftSpeed / tempHighestSpeed;
            frontRightSpeed = frontRightSpeed / tempHighestSpeed;
            backLeftSpeed = backLeftSpeed / tempHighestSpeed;
            backRightSpeed = backRightSpeed / tempHighestSpeed;
        } */
        //
        backRight.drive(backRightSpeed, backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        frontLeft.drive(frontLeftSpeed, frontLeftAngle);

    }

    void driveRotateAroundPoint(double joystickForward, double joystickSideways, double joystickTurn){
        //TODO change variable to convert joystick axis to desired velocity in meters per second
        double joystickToVelocity = 1; 
        ChassisSpeeds robotSpeed = ChassisSpeeds(joystickForward * joystickToVelocity , joystickSideways * joystickToVelocity, joystickTurn * joystickToVelocity);
        
        Double[] swerveModuleStates = kinematics.toSwerveModuleStates(robotSpeed, rotationPoint);
        //
        backRight.drive(SwerveModuleStates[0], backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        frontLeft.drive(frontLeftSpeed, frontLeftAngle);

    }

    void driveTeleop(double joystickForward, double joystickSideways, double joystickTurning) {
        if (joystickTurning >= -0.03 && joystickTurning <= 0.03) {
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -3 && YawError <= 3) {
                YawError = 0;
            }
            joystickTurning = coerceToRange((YawError) * 0.010, -1, 1);
            // 0.020
        } else {
            desiredYaw = getGyroRobotYaw() + (joystickTurning * 10);
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -2 && YawError <= 2) {
                YawError = 0;
            }
            joystickTurning = coerceToRange((YawError) * 0.07, -1, 1);
        }

        if (joystickForward >= -0.01 && joystickForward <= 0.01) {
            joystickForward = 0;
        } else {
            joystickForward = joystickForward;
        }

        if (joystickSideways >= -0.01 && joystickSideways <= 0.01) {
            joystickSideways = 0;
        } else {
            joystickSideways = joystickSideways;
        }

        drive(joystickForward, joystickSideways, joystickTurning);

    }

    // odometery drive to posistion
    public void driveToPosition() {
        double tempHighestDirection;
        // Computes turning value
        // Desired Yaw
        YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
        if (YawError >= -1 && YawError <= 1) {
            YawError = 0;
        }
        turning = coerceToRange((YawError) * 0.015, -1, 1);

        // Desired Y
        YError = desiredY - robotPose2d.getY();
        if (YError >= -0.01 && YError <= 0.01) {
            YError = 0;
        }

        // Desired x
        XError = desiredX - robotPose2d.getX();
        if (XError >= -0.01 && XError <= 0.01) {
            XError = 0;
        }

        // ***
        // If a speed is more than 100% power scale the speeds down
   /*      tempHighestDirection = Math.max(Math.abs(YError), Math.abs(XError));
        if (tempHighestDirection > 1) {
            YError = YError / tempHighestDirection;
            XError= XError / tempHighestDirection;
        } */
        strafe = YError;
        forward = XError;

        drive(forward * 0.5, strafe * 0.5, turning * 0.7);

    }

    // odometery drive to posistion two
    public void driveToPositionTwo() {
        double tempHighestDirection;
        // Computes turning value
        // Desired Yaw
        YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
        if (YawError >= -3 && YawError <= 3) {
            YawError = 0;
        }
        turning = coerceToRange((YawError) * 0.015, -1, 1);

        // Desired Y
        YError = desiredY - robotPose2d.getY();
        if (YError >= -0.01 && YError <= 0.01) {
            YError = 0;
        }

        // 1.2

        // Desired x
        XError = desiredX - robotPose2d.getX();
        if (XError >= -0.01 && XError <= 0.01) {
            XError = 0;
        }
        // 1.2

        // ***
        // If a speed is more than 100% power scale the speeds down
        /* tempHighestDirection = Math.max(Math.abs(YError), Math.abs(XError));
        if (tempHighestDirection > 1) {
            YError = YError / tempHighestDirection;
            XError= XError / tempHighestDirection;
        } */
        strafe = YError;
        forward = XError;
        drive(forward * 0.5, strafe * 0.5, turning * 0.3);

    }

    // drive method
    public void robotOrientedDrive(double forwardSpeed, double strafeSpeed, double turningSpeed) {
        double tempHighestSpeed;
        diagonal = Math.sqrt((length * length) + (width * width));

        if (turningSpeed >= -0.03 && turningSpeed <= 0.03) {
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -3 && YawError <= 3) {
                YawError = 0;
            }
            turningSpeed = coerceToRange((YawError) * 0.020, -1, 1);
        } else {
            desiredYaw = getGyroRobotYaw() + (turningSpeed * 10);
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -2 && YawError <= 2) {
                YawError = 0;
            }
            turningSpeed = coerceToRange((YawError) * 0.07, -1, 1);
        }

        if (forwardSpeed >= -0.01 && forwardSpeed <= 0.01) {
            forwardSpeed = 0;
        } else {
            forwardSpeed = forwardSpeed;
        }

        if (strafeSpeed >= -0.01 && strafeSpeed <= 0.01) {
            strafeSpeed = 0;
        } else {
            strafeSpeed = strafeSpeed;
        }

        diagonal = Math.sqrt((length * length) + (width * width));

        // ***
        // Adjusts values to field oriented drive
        gyro_degrees = getGyroRobotYaw();
        gyro_radians = degreesToRadians(getGyroRobotYaw());
        temp = forwardSpeed * Math.cos(gyro_radians) + strafeSpeed * Math.sin(gyro_radians);
        strafeSpeed = strafeSpeed * Math.cos(gyro_radians) - forwardSpeed * Math.sin(gyro_radians);
        forwardSpeed = temp;

        /*
         * temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
         * strafe = (forward * -1) * Math.sin(gyro_radians) + strafe *
         * Math.cos(gyro_radians);
         * forward = temp;
         */

        // ***
        double a = strafeSpeed - turningSpeed * (length / diagonal); // back horizontal
        double b = strafeSpeed + turningSpeed * (length / diagonal); // front horizontal
        double c = forwardSpeed + turningSpeed * (width / diagonal); // right vertical
        double d = forwardSpeed - turningSpeed * (width / diagonal); // left vertical

        // Speed Values
        // ***
        double backRightSpeed = Math.hypot(a, c);
        double backLeftSpeed = Math.hypot(a, d);
        double frontRightSpeed = Math.hypot(b, c);
        double frontLeftSpeed = Math.hypot(b, d);
        // Angle Values
        double backRightAngle = (Math.atan2(a, c) / Math.PI * 180);
        double backLeftAngle = (Math.atan2(a, d) / Math.PI * 180);
        double frontRightAngle = (Math.atan2(b, c) / Math.PI * 180);
        double frontLeftAngle = (Math.atan2(b, d) / Math.PI * 180);

        // ***
        // If a speed is more than 100% power scale the speeds down
        tempHighestSpeed = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
                Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)));
        if (tempHighestSpeed > 1) {
            frontLeftSpeed = frontLeftSpeed / tempHighestSpeed;
            frontRightSpeed = frontRightSpeed / tempHighestSpeed;
            backLeftSpeed = backLeftSpeed / tempHighestSpeed;
            backRightSpeed = backRightSpeed / tempHighestSpeed;
        }
        //
        backRight.drive(backRightSpeed, backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        frontLeft.drive(frontLeftSpeed, frontLeftAngle);

    }

    //
    public void setDesiredYaw(double desiredAngle) {
        desiredYaw = desiredAngle;
    }

    // this method is to convert the position on the field that we want to be
    // pointing at to the desired yaw
    // Will point the intake towards it
    public void setIntakeTowardsPoint(Pose2d desiredPoint) {
        Rotation2d pointAngle = new Rotation2d(desiredPoint.getX() - robotPose2d.getX(),
                desiredPoint.getY() - robotPose2d.getY());
        desiredYaw = pointAngle.getDegrees();
    }

    // Same as above, but will point the shooter instead
    public void setShooterTowardsPoint(Pose2d desiredPoint) {
        Rotation2d pointAngle = new Rotation2d(desiredPoint.getX() - robotPose2d.getX(),
                desiredPoint.getY() - robotPose2d.getY());
        desiredYaw = angleSubtractor(pointAngle.getDegrees(), 180);
    }

    public double returnDesiredYaw() {
        return desiredYaw;
    }

    public double returnForward() {
        return forward;
    }

    public double returnStrafe() {
        return strafe;
    }

    public double returnTurning() {
        return turning;
    }

    public String returnRobotPose2d() {
        return robotPose2d.toString();
    }

    public Rotation2d returnRotation() {
        return robotPose2d.getRotation();
    }

    public double returnX() {
        return robotPose2d.getX();
    }

    public double returnY() {
        return robotPose2d.getY();
    }

    public double returnDesiredX() {
        return desiredX;
    }

    public double returnDesiredY() {
        return desiredY;
    }

    public double returnXError() {
        return desiredX - robotPose2d.getX();
    }

    public double returnYError() {
        return desiredY - robotPose2d.getY();
    }

    public double compareFieldPositions(double firstX, double firstY, double secondX, double secondY) {
        return Math.hypot(firstX - secondX, firstY - secondY);
    }

    public void resetPosition(SwerveModulePosition[] wheelPositions, Pose2d cameraposeInTargetspace,
        Pose2d robotPose2dInFieldspace, double tv) {
        // cameraposeInTargetspace here has y as make sense y not targetspace y
        
        // if ((distanceFromTarget <= (180 * 0.0254)) && (robotPose2d.getX() -
        // pose.getX()) <= 678 && (robotPose2d.getY() - pose.getY()) <= 567 &&
        // getGyroRobotYaw() - pose.getRotation().getDegrees() >= 5678){
        // if robot is less than 120 inches from target, and robot is more than 7
        // degrees off from line normal to target, and target is seen
        SmartDashboard.putNumber("distance from target", Math.hypot(cameraposeInTargetspace.getX(), cameraposeInTargetspace.getY()));
        SmartDashboard.putNumber("angle from target", Math.abs(angleSubtractor(radiansToDegrees(Math.atan2(cameraposeInTargetspace.getX(), cameraposeInTargetspace.getY())), 180)));
        SmartDashboard.putBoolean("valid target", tv == 1);
        
        if(Math.hypot(cameraposeInTargetspace.getX(), cameraposeInTargetspace.getY()) <= (120 * 0.0254) && Math.abs(angleSubtractor(radiansToDegrees(Math.atan2(cameraposeInTargetspace.getX(), cameraposeInTargetspace.getY())), 180)) >= 7 && tv == 1){      
            // our new x and y pose equal the values from the limelight
            newX = robotPose2dInFieldspace.getX();
            newY = robotPose2dInFieldspace.getY();
            // System.out.println(newX);
            // System.out.println(newY);
            //System.out.println("updated postion by april tag");
            SmartDashboard.putString("apriltag update", "true");
            odometry.resetPosition(new Rotation2d(degreesToRadians(getGyroRobotYaw())), wheelPositions,
                    new Pose2d(newX, newY, new Rotation2d(degreesToRadians(getGyroRobotYaw()))));
        } else {
            SmartDashboard.putString("apriltag update", "false");
            // No change in the robots x and y posistion
        }

        

    }

    public void setPosition(double gyroYaw, SwerveModulePosition[] wheelPositions, Pose2d pose2d) {
        odometry.resetPosition(new Rotation2d(degreesToRadians(gyroYaw)), wheelPositions, pose2d);
    }

    public void setDesiredPosistion(double targetX, double targetY, double targetYaw) {
        desiredX = targetX;
        desiredY = targetY;
        desiredYaw = targetYaw;
    }

    Pose2d returnOdometry() {
        return robotPose2d;
    }

    void offDrive() {
        backRight.drive(0, 0);
        backLeft.drive(0, 0);
        frontRight.drive(0, 0);
        frontLeft.drive(0, 0);
    }

    void periodicOdometry() {
        // Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(degreesToRadians(getGyroRobotYaw()));
        // Update the pose
        robotPose2d = odometry.update(gyroRotation2d,
                new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                        backRight.getPosition() });
    }

    void pointToSpeaker() {
        Pose2d speakerPos = PositionHelpers.getSpeakerPosition();
        setShooterTowardsPoint(speakerPos);
        
    }

    void rotateAroundPointReturnModuleStates(ChassisSpeeds robotSpeeds, Translation2d rotationPoint){
        //ChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond);
        kinematics.toSwerveModuleStates(robotSpeeds, rotationPoint);
        //kinematics.toChassisSpeeds(frontLeft.getState(frontLeft.returnModuleSpeed(), new Rotation2d(frontLeft.returnsetPointAngle())), frontRight.getState(frontRight.returnModuleSpeed(), new Rotation2d(frontLeft.returnsetPointAngle())),
        //backLeft.getState(backLeft.returnModuleSpeed(), new Rotation2d(frontLeft.returnsetPointAngle())), backRight.getState(backRight.returnModuleSpeed(), new Rotation2d(backRight.returnsetPointAngle())));
        
    }

}
