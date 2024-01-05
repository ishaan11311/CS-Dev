package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpenCV.TeamElementSubsystem;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous")
public class Autonomous extends LinearOpMode {
    IMU imu;
    /* Declare OpMode members. */
    Robot                   myRobot = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }

    @Override
    public void runOpMode() {

        HardwareStart();

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);
            telemetry.addData("Object", "Passed waitForStart");
            telemetry.addData("ColorZone", element_zone);
            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");
        telemetry.addData("ColorZone", element_zone);
        telemetry.update();

//        /*
//         * Initialize the drive system variables.
//         */
//
//        DcMotor DcleftFront;
//        DcMotor DcrightFront;
//        DcMotor DcleftBack;
//        DcMotor DcrightBack;
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//
////        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        //double yaw = orientation.getYaw(AngleUnit.DEGREES);
//
//        DcleftFront  = hardwareMap.get(DcMotor.class,"motor1");
//        DcrightFront = hardwareMap.get(DcMotor.class, "motor2");
//        DcleftBack = hardwareMap.get(DcMotor.class,"motor3");
//        DcrightBack = hardwareMap.get(DcMotor.class,"motor4");
//
//        DcleftFront.setDirection(DcMotor.Direction.REVERSE);
//        DcleftBack.setDirection(DcMotor.Direction.REVERSE);
//
//        DcleftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        DcrightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        DcleftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        DcrightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//
//        myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        //encoderDrive(DRIVE_SPEED, DcleftFront, DcrightFront, DcleftBack, DcrightBack,10, 10, 10, 10);
//        turnLeft(orientation, DcleftFront, DcleftBack, DcrightFront, DcrightBack, 90, 0.2);
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//        telemetry.update();
////        telemetry.addData("Pitch (Z)", "%.2f Deg. (Heading)", orientation.getPitch(AngleUnit.DEGREES));
////        telemetry.update();
////        telemetry.addData("Roll (Z)", "%.2f Deg. (Heading)", orientation.getRoll(AngleUnit.DEGREES));
////        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                             double leftFront, double rightFront, double leftBack, double rightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int)(leftFront * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int)(rightFront * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int)(leftBack * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int)(rightBack * COUNTS_PER_INCH);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(speed));
            DcrightFront.setPower(Math.abs(speed));
            DcleftBack.setPower(Math.abs(speed));
            DcrightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (DcleftFront.isBusy() && DcrightFront.isBusy()
                            && DcleftBack.isBusy() && DcrightBack.isBusy())) {
            }

            // Stop all motion;
            DcleftFront.setPower(0);
            DcrightFront.setPower(0);
            DcleftBack.setPower(0);
            DcrightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void turnLeft(YawPitchRollAngles orientation, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                         double degrees, double power){
        myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();
        while(opModeIsActive() && (orientation.getYaw(AngleUnit.DEGREES) > -degrees)) {
            //while(opModeIsActive()) {
            DcleftFront.setPower(-power);
            DcleftBack.setPower(power);
            DcrightFront.setPower(-power);
            DcrightBack.setPower(power);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("Pitch (Z)", "%.2f Deg. (Heading)", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("Roll (Z)", "%.2f Deg. (Heading)", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.update();
        }

        DcleftFront.setPower(0);
        DcleftBack.setPower(0);
        DcrightFront.setPower(0);
        DcrightBack.setPower(0);
    }

}
