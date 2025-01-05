package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;

@Autonomous(name = "EA Auto-Net", group = "Auto")
public final class EAAutoNet extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ErasmusRobot robot = new ErasmusRobot(this) ;
        Pose2d beginPose = new Pose2d(12, 62, Math.toRadians(90));
        robot.gripperShouldItClose =false;
        robot.updateGripper();

        waitForStart(); // ==========================================================================

            Actions.runBlocking(
                new ParallelAction(
                    robot.drive.actionBuilder(beginPose)
                            .setReversed(true)
                            .splineTo(new Vector2d(10, 28), Math.toRadians(-90))  //place specimen 1
                            .build(),
                    robot.goToNetPosition()
                )
            );
            while (opModeIsActive()){}

    }
}
