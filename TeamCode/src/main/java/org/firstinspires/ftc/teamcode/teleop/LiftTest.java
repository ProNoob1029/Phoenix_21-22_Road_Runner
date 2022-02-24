package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DcMotorServo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@TeleOp(group = "drive")
public class LiftTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorServo lift = new DcMotorServo(hardwareMap,"intake2",13.79f,28);
        Servo panta = hardwareMap.get(Servo.class, "panta"); // 0.35 - 0.85

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float liftPosition = 0;
        double pantaPosition = 0.35;
        int position = 0;
        int level1 = 450;
        int level2 = 950;
        int level3 = 1350;
        waitForStart();
        ElapsedTime liftTimer = new ElapsedTime();
        liftTimer.reset();
        double lastLiftIncrease = 0;
        while (!isStopRequested()) {
            if(gamepad2.dpad_up && liftTimer.milliseconds() > lastLiftIncrease && position < 3){
                //liftPosition += 10;
                lastLiftIncrease = liftTimer.milliseconds() + 100;
                position++;
            }

            if(gamepad2.dpad_down && liftTimer.milliseconds() > lastLiftIncrease && position > 0){
                //liftPosition -= 10;
                lastLiftIncrease = liftTimer.milliseconds() + 100;
                position--;
            }

            switch (position){
                case 0:
                    liftPosition = 0;
                    break;
                case 1:
                    liftPosition = level1;
                    break;
                case 2:
                    liftPosition = level2;
                    break;
                case 3:
                    liftPosition = level3;
                    break;
                default:
                    liftPosition = 0;
                    break;
            }

            lift.setAngle(-liftPosition,0.5f);

            telemetry.addData("encoder",lift.motor.getCurrentPosition());
            telemetry.addData("angle",liftPosition);

            //panta.setPosition(Math.max(gamepad2.left_stick_y, 0.35));

            pantaPosition = 0.35 + 0.5 * (-lift.motor.getCurrentPosition() - 200) / 250.0;

            panta.setPosition(Math.min(0.85, Math.max(pantaPosition, 0.35)));

            telemetry.addData("pantaPosition",Math.min(0.85, Math.max(pantaPosition, 0.35)));

            telemetry.update();
        }
    }
}
