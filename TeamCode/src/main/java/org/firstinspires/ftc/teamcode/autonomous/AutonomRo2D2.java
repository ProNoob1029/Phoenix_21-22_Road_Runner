package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.OmniSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "ro2d2")
public class AutonomRo2D2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(isStopRequested() == false){
            double[] vals = OmniSimple.calculateAndSet(33/4,0,0);

            drive.leftFront.setPower(vals[0]);
            drive.rightFront.setPower(vals[1]);
            drive.leftRear.setPower(vals[2]);
            drive.rightRear.setPower(vals[3]);
        }

//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        0,       //left_stick_y
//                        0.2,         //left_stick_x
//                        0     //right_stick_x
//                )
//        );
//
//        sleep(400);
//
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        0.4,       //left_stick_y
//                        0,         //left_stick_x
//                        0     //right_stick_x
//                )
//        );
//
//        sleep(2650);
//
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        0,       //left_stick_y
//                        0,         //left_stick_x
//                        0     //right_stick_x
//                )
//        );

    }
}
