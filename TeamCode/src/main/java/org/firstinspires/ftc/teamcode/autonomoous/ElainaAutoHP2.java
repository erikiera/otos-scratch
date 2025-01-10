package org.firstinspires.ftc.teamcode.autonomoous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;

@Autonomous(name = "Elaina AutoHP2", group = "Auto Test")
public final class ElainaAutoHP2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: EA - changes positionX to -24 to be closer to the samples and avoid the submersible
        Pose2d beginPose = new Pose2d(-24, -62, Math.toRadians(-90));
        ErasmusRobot robot = new ErasmusRobot(this, beginPose) ;

        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(-90)), Math.toRadians(180)) // sample 1
                        .setReversed(true)
                        .splineTo(new Vector2d(-54, 52), Math.toRadians(90)) //hp zone
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-59, 12, Math.toRadians(-90)), Math.toRadians(180)) // sample 2
                        .setReversed(true)
                        .splineTo(new Vector2d(-59, 52), Math.toRadians(90)) // hp zone
                        .setReversed(false)
                        // TODO: EA - changed positionX to 65 to avoid wall
                        .splineToLinearHeading(new Pose2d(-65, 12, Math.toRadians(-90)), Math.toRadians(180)) // sample 3
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
                        .build()
        );

    }
}
