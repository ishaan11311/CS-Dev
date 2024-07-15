/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.*;
import org.firstinspires.ftc.teamcode.Robot;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name = "DriverControl", group = "stuff")

public class DriverControl extends OpMode {
    /* Declare OpMode members. */
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    DcMotor intake;
    DcMotor liftMotor;

    CRServo tapeMeasureServo;
    Servo planeLauncherServo;
    CRServo outtakeWheelServo;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Servo elbowServo;
    Servo wristServo;
    Servo outtakeDoorServo;


    Robot myRobot = new Robot();
    ElapsedTime mStateTime = new ElapsedTime();
    boolean outtakeArmUp = false;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intake = hardwareMap.get(DcMotor.class,"intake");
        slideMotor1 = hardwareMap.get(DcMotor.class,"slide_motor1");
        slideMotor2 = hardwareMap.get(DcMotor.class,"slide_motor2");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeWheelServo = hardwareMap.get(CRServo.class, "outtakeWheel");
        wristServo = hardwareMap.get(Servo.class, "outtakeWrist");
        elbowServo = hardwareMap.get(Servo.class, "outtakeElbow");
        slideMotor1.setDirection(DcMotor.Direction.REVERSE);
        outtakeDoorServo = hardwareMap.get(Servo.class, "outtakeDoor");
        slideMotor1.setZeroPowerBehavior(BRAKE);
        slideMotor2.setZeroPowerBehavior(BRAKE);
        planeLauncherServo=hardwareMap.get(Servo.class,"planeLauncher");
        myRobot.LoadPlane(planeLauncherServo);
        myRobot.OuttakeArmRest(elbowServo, wristServo, outtakeDoorServo, outtakeWheelServo);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double xDistance = 0;
        double yDistance = 0;
        double speed = 0;
        double direction = 0;
        int preciseSpeedDivider = 4;
        boolean preciseDriving;

        //drivetrain
        if (gamepad1.left_stick_x < 0){
            xDistance = -gamepad1.left_stick_x;
        }
        else {
            xDistance = gamepad1.left_stick_x;
        }

        if (gamepad1.left_stick_y < 0) {
            yDistance = -gamepad1.left_stick_y;
        }
        else {
            yDistance = gamepad1.left_stick_y;
        }

        if (gamepad1.right_trigger > 0) {
            speed = (xDistance + yDistance) / preciseSpeedDivider;
            preciseDriving = true;
        }
        else {
            speed = xDistance + yDistance;
            preciseDriving = false;
        }

        // 57.29577951 = 1 radian = 180/pi
        if ((Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) * 57.29577951) < 0){
            direction = (Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)) * 57.29577951;
            direction += 360;
        }
        else {
            direction = (Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)) * 57.29577951;
        }

        double joystickDriftTolerance = 0.05;
        if(Math.abs(speed) > joystickDriftTolerance) {
            drive(direction, speed, gamepad1.right_stick_x * speed, -gamepad1.right_stick_x * speed, preciseDriving);
        }
        else {
            double turningSpeed = gamepad1.right_stick_x / 1.2;
            turn(turningSpeed);
        }

        if (gamepad1.right_bumper){
            intake.setPower(1);
            outtakeWheelServo.setPower(-1);
        }
        else if (gamepad1.left_bumper){
            intake.setPower(-1);
            outtakeWheelServo.setPower(1);
        }
        else if (gamepad2.a){
            myRobot.OuttakeWheelOn(outtakeWheelServo);
        }
        else if (gamepad2.b){
            myRobot.OuttakeWheelReverse(outtakeWheelServo);
        }
        else if(gamepad2.right_bumper) {
            outtakeWheelServo.setPower(0.2);
        }
        else if(gamepad2.left_bumper) {
            outtakeWheelServo.setPower(-0.2);
        }
        else{
            intake.setPower(0);
            myRobot.OuttakeWheelOff(outtakeWheelServo);
        }

        if (gamepad1.x) {
            myRobot.LoadPlane(planeLauncherServo);
        }
        if (gamepad1.y){
             myRobot.LaunchPlane(planeLauncherServo);
        }

        //Set the power of the slide motors to the value of the right stick on gamepad2
        if (gamepad2.right_stick_y != 0) {
            slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor1.setPower(gamepad2.right_stick_y);
            slideMotor2.setPower(gamepad2.right_stick_y);
        }
        else {
            slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);
        }

        //Code to set outtake arm to rest position
        if(gamepad2.dpad_down) {

            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontMotor.setTargetPosition(200);
            rightFrontMotor.setTargetPosition(200);
            leftBackMotor.setTargetPosition(200);
            rightBackMotor.setTargetPosition(200);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontMotor.setPower(0.8);
            rightFrontMotor.setPower(0.8);
            leftBackMotor.setPower(0.8);
            rightBackMotor.setPower(0.8);

            while (leftFrontMotor.isBusy()) {

            }

            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            slideMotor1.setTargetPosition(0);
            slideMotor2.setTargetPosition(0);

            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor1.setPower(1.0);
            slideMotor2.setPower(1.0);

            while (slideMotor1.isBusy() && slideMotor2.isBusy()){
                telemetry.addData("Slides going down", "");
                telemetry.addData("slideMotor1", slideMotor1.getCurrentPosition());
                telemetry.addData("slideMotor2", slideMotor2.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("slides all the way down", "");
            telemetry.addData("slideMotor1 end position", slideMotor1.getCurrentPosition());
            telemetry.addData("slideMotor2 end position", slideMotor2.getCurrentPosition());
            telemetry.update();

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.

            slideMotor1.setPower(0.0);
            slideMotor2.setPower(0.0);

            myRobot.OuttakeArmRest(elbowServo, wristServo, outtakeDoorServo, outtakeWheelServo);

        }
        //Code to set outtake arm to activated position
        if(gamepad2.dpad_up) {
            elbowServo.setPosition(0.4);
            wristServo.setPosition(0.4);

//            elbowServo.setPosition(0.64);
//            wristServo.setPosition(0.1);


//            if (outtakeArmUp == false){
//                elbowServo.setPosition(0.73);
//                mStateTime.reset();
//                outtakeArmUp = true;
//            }
//            if (outtakeArmUp == true && mStateTime.time() >= 0.1){
//                wristServo.setPosition(0.1);
//                outtakeArmUp = false;
//            }
        }

        //Close outtake door
//        if(gamepad2.left_bumper) {
//            myRobot.OuttakeDoorClosed(outtakeDoorServo, outtakeWheelServo);
//        }
        //Open outtake door
        if(gamepad2.right_bumper) {
            myRobot.OuttakeDoorOpen(outtakeDoorServo);

        }

        else {
            myRobot.OuttakeDoorClosed(outtakeDoorServo);
        }
        //telemetry.addData("leftRear", (rightRearDistance.getDistance(DistanceUnit.CM)));
        //telemetry.addData("rightRear", (rightRearDistance.getDistance(DistanceUnit.CM)));
        if (gamepad1.left_trigger>0.1) {
            liftMotor.setPower(gamepad1.left_trigger);
        }
        else if (gamepad1.dpad_down){
            liftMotor.setPower (-1);
        }
        else if (gamepad1.dpad_up){
            liftMotor.setPower (1);
        }
        else{
            liftMotor.setPower (0);
        }
        if (gamepad1.a){
            wristServo.setPosition(1);
        }
        if (gamepad1.b){
            wristServo.setPosition(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //myRobot.OuttakeArmRest(elbowServo, wristServo, outtakeDoorServo, outtakeWheelServo);
    }
    public void drive (double direction, double speed, double leftOffset, double rightOffset, boolean preciseDrive){
        leftFrontMotor  = hardwareMap.get(DcMotor.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotor.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotor.class,"RB");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(BRAKE);
        leftBackMotor.setZeroPowerBehavior(BRAKE);
        rightFrontMotor.setZeroPowerBehavior(BRAKE);
        rightBackMotor.setZeroPowerBehavior(BRAKE);

        double difference = 0;
        double lf = 0;
        double rf = 0;
        double lb = 0;
        double rb = 0;

        if(direction >= 0 && direction < 45){
            difference = 10 - direction / 4.5;
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = speed;
            lf = speed;
        }
        else if(direction >= 45 && direction < 90){
            difference = (direction - 45) * (1 / -4.5);
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = speed;
            lf = speed;
        }
        else if(direction >= 90 && direction < 135){
            difference = 10 - (direction - 90) / 4.5;
            rf = -speed;
            lb = -speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }
        else if(direction >= 135 && direction < 180){
            difference = (direction - 135) * (1 / -4.5);
            rf = -speed;
            lb = -speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }
        else if(direction >= 180 && direction < 225){
            difference = -10 - (direction - 180) / -4.5;
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = -speed;
            lf = -speed;
        }
        else if (direction >= 225 && direction < 270){
            difference = (direction-225) * (1 / 4.5);
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = -speed;
            lf = -speed;
        }
        else if (direction >= 270 && direction < 315){
            difference = -10 - (direction - 270) / -4.5;
            rf = speed;
            lb = speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }
        else {
            difference = (direction - 315) * (1 / 4.5);
            rf = speed;
            lb = speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }

        rf = rf + rightOffset;
        rb = rb + rightOffset;
        lf = lf + leftOffset;
        lb = lb + leftOffset;




            leftFrontMotor.setPower(lf);
            leftBackMotor.setPower(lb);
            rightFrontMotor.setPower(rf);
            rightBackMotor.setPower(rb);


    }
    public void turn ( double speed){

        leftFrontMotor  = hardwareMap.get(DcMotor.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotor.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotor.class,"RB");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed*-1);
        rightBackMotor.setPower(speed*-1);

    }
}