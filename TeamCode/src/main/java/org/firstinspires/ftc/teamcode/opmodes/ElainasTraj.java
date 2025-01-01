package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "ElainasTraj", group = "Auto Test")
public final class ElainasTraj extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, -62, Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-12, -62, Math.toRadians(90)));
            ErasmusRobot robot = new ErasmusRobot(this) ;

            waitForStart();

            Actions.runBlocking(
                    
                    drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(-12, -30), Math.toRadians(90))  //place specimen 1
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, -50), Math.toRadians(-90))  // back up
                        .setReversed(false)
                        .splineTo(new Vector2d(-48, -36), Math.toRadians(90))  // get sample 1
                        .setReversed(true)
                        .splineTo(new Vector2d(-56, -58), Math.toRadians(-90))  // score sample 1
                        .setReversed(false)
                        .splineTo(new Vector2d(-58, -36), Math.toRadians(90))  // get sample 2
                        .setReversed(true)
                        .splineTo(new Vector2d(-58, -58), Math.toRadians(-90))  // score sample 2
                        .setReversed(false)
                        .splineTo(new Vector2d(-64, -36), Math.toRadians(90))  // get sample 3
                        .setReversed(true)
                        .splineTo(new Vector2d(-60, -58), Math.toRadians(-90))  // score sample 3
                        // part 2
                        .setReversed(false)
                        .splineTo(new Vector2d(-12, -30), Math.toRadians(90))  // get sample 4
                        .setReversed(true)
                        .splineTo(new Vector2d(-58, -58), Math.toRadians(-90))  // score sample 4
                        .setReversed(false)
                        .splineTo(new Vector2d(-12, -30), Math.toRadians(90))  // get sample 5
                        .setReversed(true)
                        .splineTo(new Vector2d(-58, -58), Math.toRadians(-90))  // score sample 5
                        .setReversed(false)
                        .splineTo(new Vector2d(-12, -30), Math.toRadians(90))  // get sample 6
                        .setReversed(true)
                        .splineTo(new Vector2d(-58, -58), Math.toRadians(-90))  // score sample 6
                        .setReversed(false)
                        .splineTo(new Vector2d(-12, -30), Math.toRadians(90))  // get sample 7
                        .setReversed(true)
                        .splineTo(new Vector2d(-58, -58), Math.toRadians(-90))  // score sample 7 and park and park  
                        .build());
        } else {
            throw new RuntimeException();
        }
    }
}
