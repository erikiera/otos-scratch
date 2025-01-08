package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepAutoHP {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d beginPose = new Pose2d(-12, 62, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-10, 28), Math.toRadians(-90))  //place specimen 1
                        .setReversed(false)
                        .splineTo(new Vector2d(-24, 48), Math.toRadians(180))
                        .splineTo(new Vector2d(-36, 22), Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(-90)), Math.toRadians(180))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-48, 56, Math.toRadians(-90)), Math.toRadians(-90))
                        // Retrieve sample 2 -----------------------------------
                        .setReversed(false)
                        .splineTo(new Vector2d(-48, 22), Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(-90)), Math.toRadians(180))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-58, 56, Math.toRadians(-90)), Math.toRadians(-90))
                        // Retrieve sample 3 -----------------------------------
                        .setReversed(false)
                        .splineTo(new Vector2d(-58, 22), Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-66, 12, Math.toRadians(-90)), Math.toRadians(180))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-66, 56, Math.toRadians(-90)), Math.toRadians(-90))
                        // Go to staging area
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-36, 46, Math.toRadians(-135)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(-26, 62, Math.toRadians(180)), Math.toRadians(90))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}