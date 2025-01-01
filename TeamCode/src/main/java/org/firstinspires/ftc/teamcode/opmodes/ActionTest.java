package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "ActionTest", group = "Auto Test")
public final class ActionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ErasmusRobot robot = new ErasmusRobot(this) ;
        Pose2d beginPose = new Pose2d(0, 0, 0);
        robot.gripperOpenState=false;
        robot.updateGripper();
        waitForStart();

            Actions.runBlocking(new ParallelAction(
                robot.drive.actionBuilder(beginPose).splineTo(new Vector2d(10, 0), 0)
                .build(),
                new SequentialAction(
                        robot.goToNetPosition(),
                        robot.goToFloorPosition()))
            );
            while (opModeIsActive()){}

    }
}
