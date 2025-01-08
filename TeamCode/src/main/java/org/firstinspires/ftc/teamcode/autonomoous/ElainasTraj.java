package org.firstinspires.ftc.teamcode.autonomoous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ErasmusRobot;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

@Autonomous(name = "ElainasTraj", group = "Auto Test")
public final class ElainasTraj extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, -62, Math.toRadians(90));
        ErasmusRobot robot = new ErasmusRobot(this, beginPose) ;

        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(beginPose)
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
                        .build()
        );

    }
}
