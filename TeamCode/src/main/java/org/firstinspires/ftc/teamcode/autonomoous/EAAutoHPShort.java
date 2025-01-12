package org.firstinspires.ftc.teamcode.autonomoous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;

@Autonomous(name = "EA Auto-HP SHORT (Right Side)", group = "Auto")
public final class EAAutoHPShort extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, 62, Math.toRadians(90));
        ErasmusRobot robot = new ErasmusRobot(this, beginPose) ;
        robot.gripperShouldItClose =true;
        robot.updateGripper();
        //Actions.runBlocking(robot.goToClipPosition());
        //robot.armServo.setPosition(ErasmusRobot.ARM_SERVO_CLIP);

        waitForStart(); // ==========================================================================


        Actions.runBlocking(
                new SequentialAction(
                        // -------- Drive preload to clip on the bar -----------
                        new ParallelAction(
                                robot.drive.actionBuilder(beginPose)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-9, 28), Math.toRadians(-90))  //place specimen 1
                                        .build(),
                                robot.goToClipPosition()
                        ),
                        // ---------- Separate from the submersible -------------
                        new ParallelAction(
                                robot.goToFloorPosition(),
                                robot.drive.actionBuilder(new Pose2d(-9, 28, Math.toRadians(90)))
                                        .splineTo(new Vector2d(-24, 42), Math.toRadians(180))
                                        .splineTo(new Vector2d(-35, 22), Math.toRadians(-90))
                                        .build()
                        ),
                        // ---------- Push the 3 samples into the observation zone -------------
                        robot.drive.actionBuilder(new Pose2d(-35, 22, Math.toRadians(-90)))
                                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(-90)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-48, 56, Math.toRadians(-90)), Math.toRadians(90))  // Push sample 1
                                // Retrieve sample 2 -----------------------------------
                                .setReversed(false)
                                .splineTo(new Vector2d(-50, 22), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(-90)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-58, 56, Math.toRadians(-90)), Math.toRadians(90))  // Push sample 2
                                // Go to staging area
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-36, 46, Math.toRadians(-135)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-25, 60, Math.toRadians(180)), Math.toRadians(90))
                                .build(),
                        // -------------------- Clip the 2 specimens -------------------------
                        // ------ Specimen 1 ------
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-25, 60, Math.toRadians(180)))
                                        .splineTo(new Vector2d(-46, 60), Math.toRadians(180))
                                        .build(),
                                robot.goToFloorPosition()
                        ),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-46, 60, Math.toRadians(180)))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-10, 28), Math.toRadians(-90))
                                        .build(),
                                robot.goToClipPosition()
                        ),
                        // ------ Specimen 2 ------
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-10, 28, Math.toRadians(90)))
                                        .splineTo(new Vector2d(-46, 60), Math.toRadians(180))
                                        .build(),
                                robot.goToFloorPosition()
                        ),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-46, 60, Math.toRadians(180)))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-10, 28), Math.toRadians(-90))
                                        .build(),
                                robot.goToClipPosition()
                        ),
                        // ----------------------- Park ------------------------
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-10, 28, Math.toRadians(90)))
                                        .splineTo(new Vector2d(-46, 60), Math.toRadians(180))
                                        .build(),
                                robot.goToFloorPosition()
                        )
                )
        );



    }
}
