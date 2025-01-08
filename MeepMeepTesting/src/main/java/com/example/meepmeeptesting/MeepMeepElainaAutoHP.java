package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepElainaAutoHP {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d beginPose = new Pose2d(-12, 62, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen 1
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-24, 36), Math.toRadians(180))
                        .splineTo(new Vector2d(-40, 12), Math.toRadians(-90))
                        .strafeTo(new Vector2d(-48,12))
                        .setReversed(true)
                        .splineTo(new Vector2d(-48, 52), Math.toRadians(90)) //hp zone
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-48, 12), Math.toRadians(-90))
                        .strafeTo(new Vector2d(-59,12))
                        .setReversed(true)
                        .splineTo(new Vector2d(-59, 52), Math.toRadians(90)) // hp zone
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-59, 12), Math.toRadians(-90)) // drive
                        .strafeTo(new Vector2d(-64,12)) //strafe right
                        .setReversed(true)
                        .splineTo(new Vector2d(-60, 52), Math.toRadians(90)) // hp zone
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-36, 40), Math.toRadians(0))
                        .splineTo(new Vector2d(-20, 48), Math.toRadians(90))
                        .splineTo(new Vector2d(-40, 52), Math.toRadians(180))
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-40, 52), Math.toRadians(180))  //place specimen
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-40, 52), Math.toRadians(180))  //place specimen
                        .setReversed(true)
                        .splineTo(new Vector2d(-8, 28), Math.toRadians(-90))  //place specimen
                        .setReversed(false)  // TODO: Added for MeepMeep
                        .splineTo(new Vector2d(-54, 54), Math.toRadians(180))  //place specimen
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}