package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.39, 49.39, Math.toRadians(152.85), Math.toRadians(60), 17.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 61.5, Math.toRadians(90)))
                                .back(31)
                                .forward(7)
                                .splineToLinearHeading(new Pose2d(40, 35, Math.toRadians(-180)), Math.toRadians(-90))
                                .back(13)
                                .forward(13)
                                .lineToConstantHeading(new Vector2d(40, 14))
                                .lineToConstantHeading(new Vector2d(60, 14))

//                                .splineToConstantHeading(new Vector2d(60, -62), Math.toRadians(20))
//                                .lineToConstantHeading(new Vector2d(40, -62))
//                                .lineToConstantHeading(new Vector2d(60, -62))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}