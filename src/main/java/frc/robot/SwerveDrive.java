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

/** Add your docs here. */
public class SwerveDrive {
    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;
    //dimensions between wheels center-to-center
    public static final double length = 22.25;
    public static final double width = 22.25;


    // Kinematics
    private static Translation2d translation2dfrontRight = new Translation2d(width / 2, -(length / 2));
    private static Translation2d translation2dfrontLeft = new Translation2d(width / 2, (length / 2));
    private static Translation2d translation2dbackRight = new Translation2d(-(width / 2), -(length / 2));
    private static Translation2d translation2dbackLeft = new Translation2d(-(width / 2), length / 2);
    
    //private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    //ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
    //ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0);
    //private final SwerveDriveOdometry odometry;

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
    double tempHighestSpeed;

    // ***
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    double yawOffset;

    // ***
    double getGyroRobotYaw() {
        return angleSubtractor(yawOffset, gyro.getYaw());
    }
    void zeroGyro(){
        yawOffset = 0;
        gyro.zeroYaw();
    }
    void setYawOffset(double newYawOffset){
        yawOffset = newYawOffset;
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(translation2dfrontLeft, translation2dfrontRight, translation2dbackLeft, translation2dbackRight);

    private final SwerveDriveOdometry odometry;
    //private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, desiredYaw, backRight.getPosition(), backLeft.getPosition(), frontRight.getPosition(), frontLeft.getPosition());

    // SwerveDrive constructor
    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft){
        // This is the position of the swerve modules on our robot

        //System.out.println("constructing swerve");
        
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;

        gyroRotation2d = new Rotation2d(0);
        robotPose2d = new Pose2d(0, 0, gyroRotation2d);
    
        
        this.odometry = new SwerveDriveOdometry(kinematics, gyroRotation2d, new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, robotPose2d);
        
        //new SwerveDriveOdometry(kinematics, desiredYaw, new SwerveModulePosition[] {backRight.getPosition(), backLeft.getPosition(), frontRight.getPosition(), frontLeft.getPosition()});

    }

    // This function converts inches to meters
    public double inchesToMeters(double inches){
        return inches * 0.0254;
    }
    // Subtracts two angles
    public double angleSubtractor (double firstAngle, double secondAngle) {
        double result = (((firstAngle - secondAngle) + 360180)%360) -180;
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
    public double coerceToRange (double number, double min, double max){
        double coercedValue;
        if (number >= max){
            coercedValue = max;
        }
        else if (number <= min){
            coercedValue = min;
        } else {
            coercedValue = number;
        }
        
        return coercedValue;
    }

    // drive method
    // ***
    public void drive (double forwardSpeed, double strafeSpeed, double turningSpeed) {
        diagonal = Math.sqrt((length * length) + (width * width));
                
        // Convert to chassis speeds
        //ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());

        //***
         // Adjusts values to field oriented drive
        gyro_degrees = getGyroRobotYaw();
        gyro_radians = degreesToRadians(getGyroRobotYaw());
        temp = forwardSpeed * Math.cos(gyro_radians) + strafeSpeed * Math.sin(gyro_radians);
        strafeSpeed = strafeSpeed * Math.cos(gyro_radians) - forwardSpeed * Math.sin(gyro_radians);
        forwardSpeed = temp;

        /* temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        strafe = (forward * -1) * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
        forward = temp; */
        
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

    void driveTeleop(double joystickForward, double joystickSideways, double joystickTurning){
        if (joystickTurning >= -0.03 && joystickTurning <=0.03){
            YawError = angleSubtractor(desiredYaw,getGyroRobotYaw());
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            joystickTurning = coerceToRange((YawError) * 0.020, -1, 1);
        } else {
            desiredYaw = getGyroRobotYaw() + (joystickTurning * 10);
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -2 && YawError <= 2){
                YawError = 0;
            } 
            joystickTurning = coerceToRange((YawError) * 0.07, -1, 1);
        }

        if (joystickForward >= -0.01 && joystickForward <= 0.01) {
            joystickForward = 0;
        } else {
            joystickForward = joystickForward;
        }

        if (joystickSideways >= -0.01 && joystickSideways <= 0.01){
            joystickSideways = 0;
        } else {
            joystickSideways = joystickSideways;
        }
         
        drive(joystickForward, joystickSideways, joystickTurning);
        

    }

    // odometery drive to posistion
    public void driveToPosition (double x1, double y1, double x2) {
        diagonal = Math.sqrt((length * length) + (width * width));
        
        /*// Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(-(getGyroRobotYaw()/360 * 2 * Math.PI));

        // Update the pose
        robotPose2d = odometry.update(gyroRotation2d,
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}); */
        
        // Convert to chassis speeds
        //ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        
        //Computes turning value
        // Desired Yaw
        YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
        if (YawError >= -3 && YawError <= 3){
            YawError = 0;
        } 
        turning = coerceToRange((YawError) * 0.015, -1, 1);

        // Desired Y
        YError = desiredY - robotPose2d.getY();
        if (YError >= -0.01 && YError <= 0.01){
            YError = 0;
        } 
        strafe = coerceToRange(YError * 1.2, -1, 1);
    
        // Desired x
        XError = desiredX - robotPose2d.getX();
        if (XError >= -0.01 && XError <= 0.01){
            XError = 0;
        } 
        forward = coerceToRange(XError * 1.2, -1, 1);

        drive(forward, strafe, turning);
        
    }

    // odometery drive to posistion two
    public void driveToPositionTwo () {
        diagonal = Math.sqrt((length * length) + (width * width));
        
        //Computes turning value
        // Desired Yaw
        YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
        if (YawError >= -3 && YawError <= 3){
            YawError = 0;
        } 
        turning = coerceToRange((YawError) * 0.015, -1, 1);

        // Desired Y
        YError = desiredY - robotPose2d.getY();
        if (YError >= -0.01 && YError <= 0.01){
            YError = 0;
        } 
        
        strafe = coerceToRange(YError, -1, 1);
        //1.2
    
        // Desired x
        XError = desiredX - robotPose2d.getX();
        if (XError >= -0.01 && XError <= 0.01){
            XError = 0;
        } 
        forward = coerceToRange(XError, -1, 1);
        //1.2
    
        drive(forward * 0.3, strafe * 0.3, turning * 0.3);
        
    }
    
    
    // drive method
    public void robotOrientedDrive (double x1, double y1, double x2) {
        diagonal = Math.sqrt((length * length) + (width * width));

        /*// Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(-(getGyroRobotYaw()/360 * 2 * Math.PI));

        // Update the pose
        robotPose2d = odometry.update(gyroRotation2d,
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}); */
        
        
        // Convert to chassis speeds
        //ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        

        if (x2 >= -0.01 && x2 <=0.01){
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            //turning = coerceToRange((YawError) * 0.015, -1, 1);
            turning = coerceToRange((YawError) * 0.020, -1, 1);
        } 
        
        else {
            desiredYaw = getGyroRobotYaw() + (x2 * 5);
            YawError = angleSubtractor(desiredYaw, getGyroRobotYaw());
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            //turning = coerceToRange((YawError) * 0.06, -1, 1);
            turning = coerceToRange((YawError) * 0.1, -1, 1);
        }


        if (y1 >= -0.01 && y1 <= 0.01) {
            forward = 0;
        } else {
            forward = y1 * -1;
        }

        if (x1 >= -0.01 && x1 <= 0.01){
            strafe = 0;
        } else {
            strafe = x1;
        }
        

        double a = strafe - turning * (length / diagonal); //back horizontal
        double b = strafe + turning * (length / diagonal); //front horizontal
        double c = forward - turning * (width / diagonal);  //right vertical
        double d = forward + turning * (width / diagonal);  //left vertical

        /*We switched left and right(we could also have switched front and back)
        * this change should turn the wheels the right way when the robot is trying to rotate
        */
        //Speed Values
        double backRightSpeed = Math.sqrt ((a * a) + (c * c));
        double backLeftSpeed = Math.sqrt ((a * a) + (d * d));
        double frontRightSpeed = Math.sqrt ((b * b) + (c * c));
        double frontLeftSpeed = Math.sqrt ((b * b) + (d * d));
        //Angle Values
        double backRightAngle = (Math.atan2(a, c) / Math.PI * 180); 
        double backLeftAngle = (Math.atan2(a, d) / Math.PI * 180);
        double frontRightAngle = (Math.atan2(b, c) / Math.PI * 180);
        double frontLeftAngle = (Math.atan2(b, d) / Math.PI * 180);

        // 
        backRight.drive(backRightSpeed, backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        frontLeft.drive(frontLeftSpeed, frontLeftAngle);
        
    }
    // 
    public void setDesiredYaw(double desiredAngle){
        desiredYaw = desiredAngle;
    }

    // this method is to convert the position on the field that we want to be pointing at to the desired yaw
    public void setTurnPoint(Pose2d desiredPoint){
        Rotation2d pointAngle = new Rotation2d(desiredPoint.getX() - robotPose2d.getX(), robotPose2d.getY() - desiredPoint.getY());
        desiredYaw = pointAngle.getDegrees();
    }

    public double returnDesiredYaw(){
        return desiredYaw;
        }

    public double returnForward(){
        return forward;
    }
    public double returnStrafe(){
        return strafe;
    }
    public double returnTurning(){
        return turning;
    }

    public String returnRobotPose2d(){
        return robotPose2d.toString();
    }

    public Rotation2d returnRotation(){
        return robotPose2d.getRotation();
    }
    public double returnX(){
        return robotPose2d.getX();
    }
    public double returnY(){
        return robotPose2d.getY();
    }
    public double returnDesiredX(){
        return desiredX;
    }
    public double returnDesiredY(){
        return desiredY;
    }
    public double returnXError(){
        return desiredX - robotPose2d.getX();
    }
    public double returnYError(){
        return desiredY - robotPose2d.getY();
    }

    public double compareFieldPositions(double firstX, double firstY, double secondX, double secondY){
       return ((firstX - secondX) * (firstX - secondX) + (firstY - secondY) * (firstY - secondY));
    }

    public void resetPosition(SwerveModulePosition[] wheelPosistions, Pose2d botposeInTargetspace, Pose2d robotPose2dInFieldspace, double tv){


        if (false){
            // Put joystick button in this case
        } else {
            //if ((distanceFromTarget <= (180 * 0.0254)) && (robotPose2d.getX() - pose.getX()) <= 678 && (robotPose2d.getY() - pose.getY()) <= 567  && getGyroRobotYaw() - pose.getRotation().getDegrees() >= 5678){
            //if robot is less than 120 inches from target, and robot is more than 7 degrees off from line normal to target, and target is seen
            if ((Math.sqrt(botposeInTargetspace.getX() * botposeInTargetspace.getX() + botposeInTargetspace.getY() * botposeInTargetspace.getY()) <= (120 * 0.0254)) && Math.abs(botposeInTargetspace.getRotation().getDegrees()) >= 7 && tv == 1){
                // our new x and y pose equal the values from the limelight
                newX = robotPose2dInFieldspace.getX();
                newY = robotPose2dInFieldspace.getY();
                //System.out.println(newX);
                //System.out.println(newY);

                odometry.resetPosition(new Rotation2d(degreesToRadians(getGyroRobotYaw())), wheelPosistions, new Pose2d(newX, newY, new Rotation2d(degreesToRadians(getGyroRobotYaw()))));
        
            } else {
                // No change in the robots x and y posistion

            }
            
        }
        
    }

    public void setPosition(double gyroYaw, SwerveModulePosition[] wheelPosistions, Pose2d pose2d){
        odometry.resetPosition(new Rotation2d(gyroYaw), wheelPosistions, pose2d);
    }

    public void setDesiredPosistion(double targetX, double targetY, double targetYaw){
        desiredX = targetX;
        desiredY = targetY;
        desiredYaw = targetYaw;
    }

    Pose2d returnOdometry(){
        return robotPose2d;
    }

    void offDrive(){
        backRight.drive(0, 0);
        backLeft.drive(0, 0);
        frontRight.drive(0, 0);
        frontLeft.drive(0, 0);
    }
    void periodicOdometry(){
        // Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(degreesToRadians(getGyroRobotYaw()));
        // Update the pose
        robotPose2d = odometry.update(gyroRotation2d,
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    }

}

