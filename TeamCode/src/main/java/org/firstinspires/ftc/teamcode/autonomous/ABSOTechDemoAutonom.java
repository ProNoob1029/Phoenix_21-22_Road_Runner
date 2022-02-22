package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class ABSOTechDemoAutonom extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        drive.setWeightedDrivePower(
                new Pose2d(
                        0,       //left_stick_y
                        0.2,         //left_stick_x
                        0     //right_stick_x
                )
        );

        sleep(500);

        drive.setWeightedDrivePower(
                new Pose2d(
                        0.4,       //left_stick_y
                        0,         //left_stick_x
                        0     //right_stick_x
                )
        );

        sleep(2500);

        drive.setWeightedDrivePower(
                new Pose2d(
                        0,       //left_stick_y
                        0,         //left_stick_x
                        0     //right_stick_x
                )
        );
    }
}
