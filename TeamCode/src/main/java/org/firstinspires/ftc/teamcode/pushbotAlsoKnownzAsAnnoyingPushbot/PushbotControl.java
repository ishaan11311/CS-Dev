package org.firstinspires.ftc.teamcode.pushbotAlsoKnownzAsAnnoyingPushbot;

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

@TeleOp(name = "PushbotControl", group = "pushbot")

/**
 * Hear yee, hear yee, tis the mighty PUSHIBOI! Sponsered by goBilda, Andydark,
 * VEX Robortics, FIRST Fish Challenge, The Red Alliance, Iceland, and the United States of America! Accessories sold separately.
 * Buy your own pushiboy today!
 * ony for 420.69*
 * * with 690% tax so YES
 * *also thats in bitcoin
 */

public class PushbotControl extends OpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor arm;
    Servo claw;
    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class,"LB");

        rightDrive = hardwareMap.get(DcMotor.class,"RB");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(BRAKE);



    }
    @Override
    public void loop() {
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        leftDrive.setPower((gamepad1.left_stick_y*-.45)+(gamepad1.right_stick_x)/2);
        rightDrive.setPower((gamepad1.left_stick_y*-.45)-(gamepad1.right_stick_x/2));
        arm.setPower(-0.25*gamepad1.left_trigger+0.40*gamepad1.right_trigger);
        if (gamepad1.left_bumper){
            claw.setPosition(.80);
        }
        else if (gamepad1.right_bumper){
            claw.setPosition(1);
        }

    }


}
