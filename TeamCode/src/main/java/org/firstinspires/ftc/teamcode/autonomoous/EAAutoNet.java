package org.firstinspires.ftc.teamcode.autonomoous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;

@Autonomous(name = "EA Auto-Net (Left Side)", group = "Auto")
public final class EAAutoNet extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(12, 62, Math.toRadians(90));
        ErasmusRobot robot = new ErasmusRobot(this, beginPose) ;
        robot.gripperShouldItClose =true;
        robot.updateGripper();

        waitForStart(); // ==========================================================================

        Actions.runBlocking(
                new SequentialAction(
                        // -------- Drive preload to clip on the bar -----------
                        new ParallelAction(
                                robot.drive.actionBuilder(beginPose)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(10, 28), Math.toRadians(-90))  //place specimen 1
                                        .build(),
                                robot.goToClipPosition()
                        ),
                        // ---------- Separate from the submersible -------------
                        new ParallelAction(
                                robot.goToFloorPosition(),
                                robot.drive.actionBuilder(new Pose2d(10, 28, Math.toRadians(90)))
                                        .splineTo(new Vector2d(32, 48), Math.toRadians(0))
                                        .splineTo(new Vector2d(48, 36), Math.toRadians(-90))
                                        .build()
                        ),
                        robot.drive.actionBuilder(new Pose2d(48, 36, Math.toRadians(-90)))
                                .splineTo(new Vector2d(48, 30), Math.toRadians(-90))  // Approach specimen 1
                                .build(),
                        new ParallelAction(
                                robot.goToNetPosition(),
                                robot.drive.actionBuilder(new Pose2d(48, 30, Math.toRadians(-90)))
                                        .waitSeconds(0.4)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(52, 52), Math.toRadians(60)) // Approach net
                                        .splineTo(new Vector2d(60, 60), Math.toRadians(45)) // Go to net
                                        .build()

                        )









            )
        );


    }
}
