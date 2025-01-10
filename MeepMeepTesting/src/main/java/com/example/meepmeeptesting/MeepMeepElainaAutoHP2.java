package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepElainaAutoHP2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d beginPose = new Pose2d(-36, 62, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(-90)), Math.toRadians(180)) // sample 1
                        .setReversed(true)
                        .splineTo(new Vector2d(-54, 52), Math.toRadians(90)) //hp zone
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-59, 12, Math.toRadians(-90)), Math.toRadians(180)) // sample 2
                        .setReversed(true)
                        .splineTo(new Vector2d(-59, 52), Math.toRadians(90)) // hp zone
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-66, 12, Math.toRadians(-90)), Math.toRadians(180)) // sample 3
                        .setReversed(true)
                        .splineTo(new Vector2d(-60, 52), Math.toRadians(90)) // hp zone

                        //score specimen
                        .splineToLinearHeading(new Pose2d(-40, 60, Math.toRadians(180)), Math.toRadians(0)) //pick up pos
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)
                        .splineTo(new Vector2d(-40, 60), Math.toRadians(180))  //pick up pos
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)
                        .splineTo(new Vector2d(-40, 60), Math.toRadians(180))  //pick up pos
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)
                        .splineTo(new Vector2d(-40, 60), Math.toRadians(180))  //pick up pos
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-54, 60, Math.toRadians(-90)), Math.toRadians(180)) //park
                        //TODO: end facing starting pos for teleop
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}