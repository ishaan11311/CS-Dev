package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    //Constants for drone
    public static double LAUNCH_DRONE = 0.70;
    public static double LOAD_DRONE = .95;

    //Constants for outtake
    public static double OUTTAKE_ARM_ELBOW_UP = .73;
    public static double OUTTAKE_ARM_ELBOW_DOWN = 0.1;
    public static double OUTTAKE_ARM_WRIST_UP = 0.1;
    public static double OUTTAKE_ARM_WRIST_DOWN = 1;
    public static double OUTTAKE_WHEEL_ON = -1.0;
    public static double OUTTAKE_WHEEL_OFF = 0.0;
    public static double OUTTAKE_WHEEL_REVERSE = 1.0;
    public static double OUTTAKE_DOOR_OPEN = 0.25;
    public static double OUTTAKE_DOOR_CLOSED = 0.5;
    //Constants for intake


    //Constants for lift
    public static double LIFT_MOTOR_PULL = 1.0;
    public static double LIFT_MOTOR_OFF = 0.0;

//don't define class names in DriverControl
    public void LaunchPlane(Servo planeLauncher){
        planeLauncher.setPosition(LAUNCH_DRONE);
    }

    public void LoadPlane(Servo planeLauncher) {
        planeLauncher.setPosition(LOAD_DRONE);
    }

    public void OuttakeArmDrop(Servo elbow, Servo wrist){
        elbow.setPosition(OUTTAKE_ARM_ELBOW_UP);
        wrist.setPosition(OUTTAKE_ARM_WRIST_UP);
    }

    public void OuttakeArmRest(Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel){
        elbow.setPosition(OUTTAKE_ARM_ELBOW_DOWN);
        wrist.setPosition(OUTTAKE_ARM_WRIST_DOWN);
        outtakeDoor.setPosition(OUTTAKE_DOOR_CLOSED);
        outtakeWheel.setPower(OUTTAKE_WHEEL_OFF);
    }

    public void OuttakeWheelOn(CRServo outtakeWheel){
        outtakeWheel.setPower(OUTTAKE_WHEEL_ON);
    }

    public void OuttakeWheelOff(CRServo outtakeWheel){
        outtakeWheel.setPower(OUTTAKE_WHEEL_OFF);
    }

    public void OuttakeWheelReverse(CRServo outtakeWheel){
        outtakeWheel.setPower(OUTTAKE_WHEEL_REVERSE);
    }

    public void OuttakeDoorOpen(Servo outtakeDoor){
        outtakeDoor.setPosition(OUTTAKE_DOOR_OPEN);
    }

    public void OuttakeDoorClosed(Servo outtakeDoor){
        outtakeDoor.setPosition(OUTTAKE_DOOR_CLOSED);
    }

    public void LiftRaise(DcMotor lift, CRServo tapeMeasure){
        lift.setPower(-0.22);
        tapeMeasure.setPower(-1.0);
    }

    public void LiftPull(DcMotor lift,CRServo tapeMeasure ){
        lift.setPower(.75);
        tapeMeasure.setPower(1.0);
    }


    /**
     * This method is currently only for driveWithEncodersAuto
     * @param DcleftFront
     * @param DcrightFront
     * @param DcleftBack
     * @param DcrightBack
     * @param mode
     */
    public void setDriveTrainMode(DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack,
                                  DcMotor DcrightBack, DcMotor.RunMode mode){
        DcleftFront.setMode(mode);
        DcrightFront.setMode(mode);
        DcleftBack.setMode(mode);
        DcrightBack.setMode(mode);
    }

}

