package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class RRTrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory goForward = drive.trajectoryBuilder(startPose)
                .forward(50)
                .build();

        Trajectory strafeLeft = drive.trajectoryBuilder(startPose)
                .strafeLeft(50)
                .build();

        Trajectory strafeRight = drive.trajectoryBuilder(startPose)
                .strafeRight(50)
                .build();

        Trajectory goBack = drive.trajectoryBuilder(startPose)
                .back(10)
                .build();

        TrajectorySequence backwards = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .build();

        boolean run = true;

        waitForStart();

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(backwards);

//        if (run){
//            drive.followTrajectory(goBack);
//        }
    }
}
