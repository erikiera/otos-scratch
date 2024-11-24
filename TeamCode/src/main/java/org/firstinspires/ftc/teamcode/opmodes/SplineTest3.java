package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "SplineTest3", group = "Auto Test")
public final class SplineTest3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

             // This autonomous opmode make a plus sign
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(20, 18), Math.PI/2)
                        .setReversed(true)
                        .splineTo(new Vector2d(40, 0), 0)
                        .setReversed(false)
                        .splineTo(new Vector2d(20, -18), 3*Math.PI/2)
                        .setReversed(true)
                        .splineTo(new Vector2d(0, 0), Math.PI)
                        .build());
        } else {
            throw new RuntimeException();
        }
    }
}
