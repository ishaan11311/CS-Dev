package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpenCV.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="twoPlusOneAutoTesting")
public class twoPlusOneAutoTesting extends LinearOpMode {
    IMU imu;
    /* Declare OpMode members. */
    Robot myRobot = new Robot();


    ElapsedTime mStateTime = new ElapsedTime();
    boolean outtakeArmUp = false;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    String parking = "corner";
    String route = "short";
    long initDelay = 0000;
    boolean addInitDelay;
    double backDropTargetDist = 1.6;


    static final double SPEED_SLOW  = 0.1;
    static final double SPEED_MODERATE = 0.4;
    static final double SPEED_FAST  = 0.7;
    static final double SPEED_FULLSPEED = 1.0;

    YawPitchRollAngles orientation;
    String curAlliance = "red";
    public int element_zone = 2;

    private TeamElementSubsystem teamElementDetection=null;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }

    @Override
    public void runOpMode() {

        //initialize imu
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(parameters);

        orientation = imu.getRobotYawPitchRollAngles();
        //YawPitchRollAngles orientation= imu.getRobotYawPitchRollAngles();;
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("Initial yaw", yaw);

        //telemetry.update();

        /*
         * Initialize the drive system variables.
         */

//        DcMotor leftFrontMotor;
//        DcMotor rightFrontMotor;
//        DcMotor leftBackMotor;
//        DcMotor rightBackMotor;
        CRServo outtakeWheel;
        DcMotor slideMotor1;
        DcMotor slideMotor2;
        Servo elbow;
        Servo wrist;
        Servo outtakeDoor;
        DcMotor intake;
        DistanceSensor DstSensor;

//        leftFrontMotor = hardwareMap.get(DcMotor.class, "LF");
//        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF");
//        leftBackMotor = hardwareMap.get(DcMotor.class, "LB");
//        rightBackMotor = hardwareMap.get(DcMotor.class, "RB");
//
//        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class,"intake");
        slideMotor1 = hardwareMap.get(DcMotor.class,"slide_motor1");
        slideMotor2 = hardwareMap.get(DcMotor.class,"slide_motor2");
        outtakeWheel = hardwareMap.get(CRServo.class, "outtakeWheel");
        wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        elbow = hardwareMap.get(Servo.class, "outtakeElbow");
        DstSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        double backDropDist;

        slideMotor1.setDirection(DcMotor.Direction.REVERSE);
        outtakeDoor = hardwareMap.get(Servo.class, "outtakeDoor");
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        myRobot.setDriveTrainMode(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("Status", "Resetting Encoders");
//        myRobot.setDriveTrainMode(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("LF encoder before start",leftFrontMotor.getCurrentPosition());
//        telemetry.addData("RF encoder before start",rightFrontMotor.getCurrentPosition());
//        telemetry.addData("LB encoder before start",leftBackMotor.getCurrentPosition());
//        telemetry.addData("RB encoder before start",rightBackMotor.getCurrentPosition());
//        telemetry.update();
//
//        sleep(1000);
        HardwareStart();

        //element_zone = teamElementDetection.ElementDetection(telemetry);

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);

            if (togglePreview && gamepad2.a){
                togglePreview = false;
                teamElementDetection.toggleAverageZone();
            }else if (!gamepad2.a){
                togglePreview = true;
            }

            if (gamepad1.y ){
                initDelay = 5000;
            }


            else if(gamepad1.a){
                initDelay = 0000;
            }

            telemetry.addData("Select initial Delay (Gamepad1 y = 5 seconds, Gamepad1 a = 0 seconds", "");
            telemetry.addData("Current initDelay:", initDelay);


            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());

            if (gamepad1.dpad_down){
                route = "short";
            }
            else if (gamepad1.dpad_up){
                route = "long";
            }

            telemetry.addData("Select route (Gamepad1 down = short, Gamepad1 up = long", "");
            telemetry.addData("Current Route Selected:", route);

            if (gamepad1.dpad_left){
                parking = "center";
            }
            else if (gamepad1.dpad_right){
                parking = "corner";
            }

            telemetry.addData("Select parking area (Gamepad1 left = center, Gamepad1 right = corner", "");
            telemetry.addData("Current parking area selected:", parking);


            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");
        telemetry.addData("ColorZone", element_zone);
        telemetry.update();

        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);

        // Create Roadrunner Trajectories

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -61.5, Math.toRadians(-90));

        TrajectorySequence Zone2Short = drive.trajectorySequenceBuilder(startPose)
                .back(31)
                .forward(27.4)
                .turn(Math.toRadians(-90))
                .back(75)
                .lineToConstantHeading(new Vector2d(40, -35))
                .build();

        TrajectorySequence Zone1Short = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .strafeRight(12)
                .back(20)
                .forward(18)
                .turn(Math.toRadians(-90))
                .back(78)
                .lineToConstantHeading(new Vector2d(40, -29))
                .build();

        TrajectorySequence Zone3Short = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .turn(Math.toRadians(-90))
                .back(4)
                .strafeRight(2)
                .forward(8)
                .strafeLeft(25)
                .back(79)
                .lineToConstantHeading(new Vector2d(40, -41.5))
                .build();

        TrajectorySequence Zone3Short_2plus2 = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .turn(Math.toRadians(-90))
                .back(4)
                .strafeRight(2)
                .addDisplacementMarker(() -> {
                    intake.setPower(0.65);
                    outtakeWheel.setPower(-1);
                })
                .lineToConstantHeading(new Vector2d(-59.5, -35.5))
                .waitSeconds(0.2)
                .back(3)
                .lineToConstantHeading(new Vector2d(-59.5, -35.5))
                .waitSeconds(0.2)
                .back(3)
                .addTemporalMarker(() -> intake.setPower(-1))
                .addTemporalMarker(() -> outtakeWheel.setPower(0))
                .waitSeconds(1)
                .back(17)
//              .splineToConstantHeading(new Vector2d(-37, -59), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-40, -58.5))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .back(80)
                .lineToConstantHeading(new Vector2d(40, -41))
                .build();

        TrajectorySequence Zone2Long = drive.trajectorySequenceBuilder(startPose)
                .back(29.5)
                .forward(7)
                .strafeRight(19)
                .back(27.5)
                .turn(Math.toRadians(-90))
                .back(94)
                .lineToConstantHeading(new Vector2d(40, -35))
                .build();

        TrajectorySequence Zone1Long = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .strafeRight(12)
                .back(20)
                .forward(10)
                .strafeLeft(14)
                .back(39)
                .turn(Math.toRadians(-90))
                .back(73)
                .lineToConstantHeading(new Vector2d(40, -29))
                .build();

        TrajectorySequence Zone3Long = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .turn(Math.toRadians(-90))
                .back(4)
                .strafeRight(2)
                .forward(10)
                .strafeRight(23)
                .back(81)
                .lineToConstantHeading(new Vector2d(40, -41.5))
                .build();


        TrajectorySequence ParkCenter = drive.trajectorySequenceBuilder(new Pose2d(40, -35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40, -14))
                .lineToConstantHeading(new Vector2d(60, -14))
                .build();

        TrajectorySequence ParkCorner = drive.trajectorySequenceBuilder(new Pose2d(40, -35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40, -62))
                .lineToConstantHeading(new Vector2d(60, -62))
                .build();

        Trajectory back13Zone1 = drive.trajectoryBuilder(Zone1Short.end())
                .back(13)
                .build();

        Trajectory forward13inZone1 = drive.trajectoryBuilder(back13Zone1.end())
                .forward(13)
                .build();

        Trajectory back13inZone2 = drive.trajectoryBuilder(Zone2Short.end())
                .back(13)
                .build();

        Trajectory forward13inZone2 = drive.trajectoryBuilder(back13inZone2.end())
                .forward(13)
                .build();

        Trajectory back13inZone3 = drive.trajectoryBuilder(Zone3Short.end())
                .back(13)
                .build();

        Trajectory forward13inZone3 = drive.trajectoryBuilder(back13inZone3.end())
                .forward(13)
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //sleep(initDelay);

        drive.setPoseEstimate(startPose);

        //drive.followTrajectorySequence(Zone2Short);

        if (route == "short") {

            telemetry.addData("started short route", "");
            telemetry.update();

            if (element_zone == 1) {

                drive.followTrajectorySequence(Zone1Short);

                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                drive.followTrajectory(back13Zone1);

                scorePix(SPEED_FAST, slideMotor1, slideMotor2, outtakeWheel);
                drive.followTrajectory(forward13inZone1);
                slidesDown(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel);


            } else if (element_zone == 2) {

                drive.followTrajectorySequence(Zone2Short);

                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                drive.followTrajectory(back13inZone2);

                scorePix(SPEED_FAST,  slideMotor1, slideMotor2, outtakeWheel);
                drive.followTrajectory(forward13inZone2);
                slidesDown(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel);


            } else if (element_zone == 3) {

                drive.followTrajectorySequence(Zone3Short_2plus2);

                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                drive.followTrajectory(back13inZone3);

                scorePix(SPEED_FAST, slideMotor1, slideMotor2, outtakeWheel);
                drive.followTrajectory(forward13inZone3);
                slidesDown(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel);


            }

            sleep(1000);

            if (parking == "corner") {
                drive.followTrajectorySequence(ParkCorner);
            }

            else if (parking == "center") {
                drive.followTrajectorySequence(ParkCenter);
            }
        }

        else if (route == "long"){
            telemetry.addData("started long route", "");
            telemetry.update();

            if (element_zone == 1) {

                drive.followTrajectorySequence(Zone1Long);

                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                drive.followTrajectory(back13Zone1);

                scorePix(SPEED_FAST, slideMotor1, slideMotor2, outtakeWheel);
                drive.followTrajectory(forward13inZone1);
                slidesDown(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel);


            } else if (element_zone == 2) {
                drive.followTrajectorySequence(Zone2Long);

                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                drive.followTrajectory(back13inZone2);

                scorePix(SPEED_FAST, slideMotor1, slideMotor2, outtakeWheel);
                drive.followTrajectory(forward13inZone2);
                slidesDown(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel);


            } else if (element_zone == 3) {

                drive.followTrajectorySequence(Zone3Long);

                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                drive.followTrajectory(back13inZone3);

                scorePix(SPEED_FAST, slideMotor1, slideMotor2, outtakeWheel);
                drive.followTrajectory(forward13inZone3);
                slidesDown(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel);

            }

            sleep(1000);

            if (parking == "corner") {
                drive.followTrajectorySequence(ParkCorner);
            }

            else if (parking == "center") {
                drive.followTrajectorySequence(ParkCenter);
            }
        }

        /**
         * This is a test
         * customEncoderDrive(DRIVE_SPEED, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor,100, 100, 100, 100);
         telemetry.addData("LF encoder after move",leftFrontMotor.getCurrentPosition());
         telemetry.addData("RF encoder after move",rightFrontMotor.getCurrentPosition());
         telemetry.addData("LB encoder after move",leftBackMotor.getCurrentPosition());++
         telemetry.addData("RB encoder after move",rightBackMotor.getCurrentPosition());
         telemetry.update();
         //sleep(5000);


         //not working//turnLeft(orientation, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, 90, 0.2);
         turnLeft( leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, 90, 0.2);
         telemetry.addData("Yaw after left turn", orientation.getYaw(AngleUnit.DEGREES));
         telemetry.update();
         //sleep(5000);

         turnRight( leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, 90, 0.2);
         telemetry.addData("Yaw after right turn", orientation.getYaw(AngleUnit.DEGREES));
         telemetry.update();
         //sleep(5000);
         */

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void customEncoderDrive(double speed, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                                   double leftFront, double rightFront, double leftBack, double rightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (leftBack * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (rightBack * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

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

    public void moveForwardEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

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

    public void moveBackwardsEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) -(inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(-speed));
            DcrightFront.setPower(Math.abs(-
                    speed));
            DcleftBack.setPower(Math.abs(-speed));
            DcrightBack.setPower(Math.abs(-speed));

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

    public void strafeRightEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(speed));
            DcrightFront.setPower(Math.abs(-speed));
            DcleftBack.setPower(Math.abs(-speed));
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

    public void strafeLeftEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(-speed));
            DcrightFront.setPower(Math.abs(speed));
            DcleftBack.setPower(Math.abs(speed));
            DcrightBack.setPower(Math.abs(-speed));

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

    public void moveSlidesTime(double power, double timeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2){
        slideMotor1.setPower(-power);
        slideMotor2.setPower(-power);

        sleep((long) timeMilliseconds);

        slideMotor1.setPower(0.0);
        slideMotor2.setPower(0.0);
    }

    public void dropPixels(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                           Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel){
        moveSlidesTime(slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep((long) slidesTimeMilliseconds + 200);
        myRobot.OuttakeArmDrop(elbow, wrist);
        sleep(1300);
        myRobot.OuttakeDoorOpen(outtakeDoor);
        myRobot.OuttakeWheelOn(outtakeWheel);
        sleep(800);
        moveSlidesTime(-slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep(1000);
        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);
    }

    public void dropPixelsV2(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                             Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel,
                             DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor){
        moveSlidesTime(slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep((long) slidesTimeMilliseconds + 200);
        outtakeArmUpV2(elbow, wrist);
        sleep(600);
        myRobot.OuttakeWheelReverse(outtakeWheel);
        sleep(800);
        moveSlidesTime(slidesPower,200, slideMotor1, slideMotor2);
        moveForwardEncoders(SPEED_MODERATE, 3, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        moveSlidesTime(-slidesPower, slidesTimeMilliseconds + 200, slideMotor1, slideMotor2);
        sleep(1000);
        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);
    }

    public void openOuttakeArm(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                               Servo elbow, Servo wrist){
        moveSlidesTime(slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep((long) slidesTimeMilliseconds + 200);
        outtakeArmUpV2(elbow, wrist);
        sleep(600);
    }

    public void scorePix(double slidesPower, DcMotor slideMotor1, DcMotor slideMotor2, CRServo outtakeWheel){
        myRobot.OuttakeWheelReverse(outtakeWheel);
        sleep(800);
        moveSlidesTime(slidesPower,200, slideMotor1, slideMotor2);
    }

    public void slidesDown(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                           Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel){
        moveSlidesTime(-slidesPower, slidesTimeMilliseconds + 200, slideMotor1, slideMotor2);
        sleep(1000);
        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);
    }

    public void turnLeft( DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                          double degrees, double power) {
        //public void turnLeft(YawPitchRollAngles orientation, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
        //                   double degrees, double power) {

        //myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();

        //orientation=imu.getRobotYawPitchRollAngles();
        //telemetry.addData("Yaw outside loop", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw outside left turn while loop", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("degree value", degrees);
        telemetry.update();
        //sleep(5000);

        while(opModeIsActive() && (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < degrees)) {
            // while(opModeIsActive() && (orientation.getYaw(AngleUnit.DEGREES) < degrees)) {
            //while(opModeIsActive()) {
            DcleftFront.setPower(-power);
            DcleftBack.setPower(-power);
            DcrightFront.setPower(power);
            DcrightBack.setPower(power);
            telemetry.addData("Yaw while turning left", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            // telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }


        DcleftFront.setPower(0);
        DcleftBack.setPower(0);
        DcrightFront.setPower(0);
        DcrightBack.setPower(0);
        //sleep(5000);
    }

    public void turnRight( DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                           double degrees, double power) {
        //public void turnLeft(YawPitchRollAngles orientation, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
        //                   double degrees, double power) {

        //myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();

        //orientation=imu.getRobotYawPitchRollAngles();
        //telemetry.addData("Yaw outside loop", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw before while loop for right turn", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Right turn degree value ", degrees);
        telemetry.update();
        //sleep(5000);

        while(opModeIsActive() && (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -degrees)) {
            // while(opModeIsActive() && (orientation.getYaw(AngleUnit.DEGREES) < degrees)) {
            //while(opModeIsActive()) {
            DcleftFront.setPower(power);
            DcleftBack.setPower(power);
            DcrightFront.setPower(-power);
            DcrightBack.setPower(-power);
            telemetry.addData("Yaw while turning right", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            // telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        DcleftFront.setPower(0);
        DcleftBack.setPower(0);
        DcrightFront.setPower(0);
        DcrightBack.setPower(0);

        telemetry.addData("final", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        //sleep(5000);
    }

    public void outtakeArmUpV2 (Servo elbow, Servo wrist){
//        if (outtakeArmUp == false){
//            elbow.setPosition(0.73);
//            mStateTime.reset();
//            outtakeArmUp = true;
//        }
//        if (outtakeArmUp == true && mStateTime.time() >= 0.1){
//            wrist.setPosition(0.1);
//            outtakeArmUp = false;
//        }

        elbow.setPosition(0.73);
        mStateTime.reset();

        while (mStateTime.time() <= 0.3){

        }

        wrist.setPosition(0.1);
    }
}