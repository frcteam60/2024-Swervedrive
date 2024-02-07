// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.DescriptorRead;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.proto.Kinematics;

// add chassis speeds
// Is our positive gyro speeds in the right angle

/** Add your docs here. */
public class SwerveDrive {
    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;
    //dimensions between wheels center-to-center
    public static final double length = 27.4;
    public static final double width = 12.25;


    // Kinematics
    private static Translation2d translation2dfrontRight = new Translation2d(width / 2, -(length / 2));
    private static Translation2d translation2dfrontLeft = new Translation2d(width / 2, (length / 2));
    private static Translation2d translation2dbackRight = new Translation2d(-(width / 2), -(length / 2));
    private static Translation2d translation2dbackLeft = new Translation2d(-(width / 2), length / 2);
    
    private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
    //ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0);
    //private final SwerveDriveOdometry odometry;

    // Gyro
    double gyro_radians;
    double gyro_degrees;

    double temp;
    double forward;
    double strafe;
    double rotation;

    // Desired Position
    double desiredX = 0;
    double desiredY = 0;
    double desiredYaw = 0;
    double XError;
    double YError;
    double YawError;
    double turning;

    Rotation2d gyroRotation2d;
    Pose2d robotPose2d;
    Pose2d fieldPose2d;
    

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(translation2dfrontLeft, translation2dfrontRight, translation2dbackLeft, translation2dbackRight);

    private final SwerveDriveOdometry odometry;
    //private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, desiredYaw, backRight.getPosition(), backLeft.getPosition(), frontRight.getPosition(), frontLeft.getPosition());

    // SwerveDrive constructor
    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft, double gyroAngle){
        // This is the position of the swerve modules on our robot
        
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;


        Rotation2d gyroRotation2d = new Rotation2d(-(gyroAngle/360 * 2 * Math.PI));
        Pose2d robotPose2d = new Pose2d(0, 0, gyroRotation2d);
    
        
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
    public void drive (double x1, double y1, double x2, double gyroAngle) {
        rotation = Math.sqrt((length * length) + (width * width));

        y1 = y1;
        
        // Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(-(gyroAngle/360 * 2 * Math.PI));


        // Update the pose
        robotPose2d = odometry.update(gyroRotation2d,
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
        
        fieldPose2d = robotPose2d.rotateBy( new Rotation2d(gyroAngle));

        
        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        
       /*/ if ((x2 > 0.1) || (x2 < -0.1)){
            desiredYaw = gyroAngle + (x2 * 10);       
        }
        
        error = angleSubtractor(desiredYaw, gyroAngle);

        if (error >= -3 && error <= 3){
            error = 0;
        }   
        turning = coerceToRange((error) * 0.02, -0.5, 0.5);*/

        if (x2 >= -0.01 && x2 <=0.01){
            YawError = angleSubtractor(desiredYaw, gyroAngle);
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            turning = coerceToRange((YawError) * 0.015, -1, 1);
        } 
        
        else {
            desiredYaw = gyroAngle + (x2 * 10);
            YawError = angleSubtractor(desiredYaw, gyroAngle);
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            turning = coerceToRange((YawError) * 0.06, -1, 1);
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

        /* */
        /*// desired X
        if (y1 >= -0.01 && y1 <= 0.01) {

            forward = 0;
        } else {
            desiredX = robotPose2d.getX() + y1;
            XError = desiredX - robotPose2d.getX();
            //forward = y1 * -1;
            forward = coerceToRange(XError, -1, 1);
        }*/

        // Desired X

        // if we aren't using the joystick
        /*/
        if (y1 >= -0.01 && y1 <=0.01){
            XError = desiredX - robotPose2d.getX();
            if (XError >= -0.03 && XError <= 0.03){
                XError = 0;
            } 
            forward = coerceToRange(XError, -1, 1);
        } else {
            // else desiredX is changed by our joystick
            desiredX = robotPose2d.getX() + (y1 * 1);
            XError = desiredX - robotPose2d.getX();
            if (XError >= -0.03 && XError <= 0.03){
                XError = 0;
            } 
            forward = coerceToRange(XError, -1, 1);
        }

        // desired Y
        if (x1 >= -0.01 && x1 <= 0.01){
            strafe = 0;
        } else {
            desiredY = robotPose2d.getX() + (x1 * 0.1);
            YError = desiredY - robotPose2d.getY();
            strafe = coerceToRange((YError), -1, 1);
        }*/


        
        // Desired Y
        // if we aren't using the joystick
        /*
        if (x1 >= -0.01 && x1 <=0.01){
            /*YError = desiredY - robotPose2d.getY();
            if (YError >= -0.03 && YError <= 0.03){
                YError = 0;
            } 
            strafe = coerceToRange(YError, -1, 1);
            strafe = 0;
        } else {
            // else desiredY is changed by our joystick
            desiredY = robotPose2d.getY() + (x1 * 1);
            YError = desiredY - robotPose2d.getY();
            if (YError >= -0.03 && YError <= 0.03){
                YError = 0;
            } 
            strafe = coerceToRange(YError, -1, 1);
        }*/



        /*rotation = Math.sqrt((length * length) + (width * width));
        forward = y1 * -1;
        strafe = x1;*/

         // Adjusts values to field oriented drive
        gyro_degrees = gyroAngle;
        gyro_radians = gyro_degrees * Math.PI/180;
        temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        strafe = (forward * -1) * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
        forward = temp;
        

        double a = strafe - turning * (length / rotation); //back horizontal
        double b = strafe + turning * (length / rotation); //front horizontal
        double c = forward - turning * (width / rotation);  //right vertical
        double d = forward + turning * (width / rotation);  //left vertical

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

    // odometery drive to posistion
    public void driveToPosistion (double x1, double y1, double x2, double gyroAngle) {
        rotation = Math.sqrt((length * length) + (width * width));

        y1 = y1;
        
        // Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(-(gyroAngle/360 * 2 * Math.PI));


        // Update the pose
        robotPose2d = odometry.update(gyroRotation2d,
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
        
        fieldPose2d = robotPose2d.rotateBy( new Rotation2d(gyroAngle));

        
        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        
        //Computes turning value
        if (x2 >= -0.01 && x2 <=0.01){
            YawError = angleSubtractor(desiredYaw, gyroAngle);
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            turning = coerceToRange((YawError) * 0.015, -1, 1);
        } 
        
        else {
            desiredYaw = gyroAngle + (x2 * 10);
            YawError = angleSubtractor(desiredYaw, gyroAngle);
            if (YawError >= -3 && YawError <= 3){
                YawError = 0;
            } 
            turning = coerceToRange((YawError) * 0.06, -1, 1);
        }

        // Desired Y
        YError = desiredY - robotPose2d.getY();
        if (YError >= -0.01 && YError <= 0.01){
            YError = 0;
        } 
        strafe = coerceToRange(YError * -1.2, -1, 1);
    
        // Desired x
        XError = desiredX - robotPose2d.getX();
        if (XError >= -0.01 && XError <= 0.01){
            XError = 0;
        } 
        forward = coerceToRange(XError * 1.2, -1, 1);
    
        // Adjusts values to field oriented drive
        gyro_degrees = gyroAngle;
        gyro_radians = gyro_degrees * Math.PI/180;
        temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        strafe = (forward * -1) * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
        forward = temp;
        

        double a = strafe - turning * (length / rotation); //back horizontal
        double b = strafe + turning * (length / rotation); //front horizontal
        double c = forward - turning * (width / rotation);  //right vertical
        double d = forward + turning * (width / rotation);  //left vertical

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
    // april tag method
    public void visionDrive (double targetX, double targetY, double targetYaw, double[] robotPosition, double gyroAngle) {
        rotation = Math.sqrt((length * length) + (width * width));
        
        // Converts gyro angle into radians then Rotation2d
        gyroRotation2d = new Rotation2d(-(gyroAngle/360 * 2 * Math.PI));
        
       /*if ((x2 > 0.1) || (x2 < -0.1)){
            desiredYaw = gyroAngle + (x2 * 10);       
        }
        
        error = angleSubtractor(desiredYaw, gyroAngle);

        if (error >= -3 && error <= 3){
            error = 0;
        }   
        turning = coerceToRange((error) * 0.02, -0.5, 0.5);*/
        
        //converts our target values to meters
        targetX = inchesToMeters(targetX);
        targetY = inchesToMeters(targetY);

        
        strafe = coerceToRange((targetY - robotPosition[1]) * -0.5, -0.5,0.5);
        forward = coerceToRange((targetX - robotPosition[0]) * -0.5, -0.5,0.5);
        turning = coerceToRange((targetYaw - robotPosition[5]) * 0.01, -0.5,0.5);
        
        /*
        if (x2 >= -0.01 && x2 <=0.01){
            error = angleSubtractor(desiredYaw, gyroAngle);
            if (error >= -3 && error <= 3){
                error = 0;
            } 
            turning = coerceToRange((error) * 0.015, -1, 1);
        } 
        
        else {
            desiredYaw = gyroAngle + (x2 * 10);
            error = angleSubtractor(desiredYaw, gyroAngle);
            if (error >= -3 && error <= 3){
                error = 0;
            } 
            turning = coerceToRange((error) * 0.06, -1, 1);
        }*/

        if (turning >= -0.03 && turning <= 0.03) {
            turning = 0;
        } else {
            turning = turning * -1;
        }

        if (forward >= -0.03 && forward <= 0.03) {
            forward = 0;
        } else {
            forward = forward * -1;
        }

        if (strafe >= -0.03 && strafe <= 0.03){
            strafe = 0;
        } else {
            strafe = strafe;
        }

        /*rotation = Math.sqrt((length * length) + (width * width));
        forward = y1 * -1;
        strafe = x1;*/

        // Adjusts values to field oriented drive
        
        /*gyro_degrees = gyroAngle;
        gyro_radians = gyro_degrees * Math.PI/180;
        temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        strafe = (forward * -1) * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
        forward = temp;*/
        

        double a = strafe - turning * (length / rotation); //back horizontal
        double b = strafe + turning * (length / rotation); //front horizontal
        double c = forward - turning * (width / rotation);  //right vertical
        double d = forward + turning * (width / rotation);  //left vertical
        

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
    public void setDesiredYaw(double desiredGyroAngle){
        desiredYaw = desiredGyroAngle;
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
    public String returnfieldPose2d(){
        return fieldPose2d.toString();
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
    public void resetPosistion(double gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose){
        odometry.resetPosition(Rotation2d.fromDegrees(-gyroAngle), modulePositions, pose);
    }

    public void updateOdometry(double gyroAngle){
        odometry.update(gyroRotation2d, new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    }
    public void setDesiredPosistion(double targetX, double targetY, double targetYaw){
        desiredX = targetX;
        desiredY = targetY;
        desiredYaw = targetYaw;
    }
    
}

