package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.OmniSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "testMotoareWeigth")
@Config
//@Disabled
public class AutonomHeterosexual extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        ElapsedTime t = new ElapsedTime();
        t.reset();

        double lLF=0,lLB=0,lRF=0,lRB=0;
        double dLF=0,dLB=0,dRF=0,dRB=0;
        Servo panta = hardwareMap.get(Servo.class, "panta");

        double measure = 0;
        while(isStopRequested() == false){
                panta.setPosition(0.55);
//            double[] vals = OmniSimple.calculateAndSet(33/4,0,0);


//
//            if(t.milliseconds() > 1500)
//            {
//                for (DcMotorEx motor :
//                        drive.motors) {
//                    motor.setPower(0);
//                }
//
//            }else{
//                for (DcMotorEx motor :
//                        drive.motors) {
//                    motor.setPower(1);
//                }
//            }

            double[] vals = OmniSimple.calculateAndSet(gamepad1.left_stick_x * 33, gamepad1.left_stick_y * 33, gamepad1.right_stick_x);

            drive.leftFront.setVelocity(vals[0], AngleUnit.RADIANS);
            drive.rightFront.setVelocity(vals[1], AngleUnit.RADIANS);
            drive.leftRear.setVelocity(vals[2] * 0.9, AngleUnit.RADIANS);
            drive.rightRear.setVelocity(vals[3] * 0.9, AngleUnit.RADIANS);

            if(measure < t.milliseconds()){
                dLF = drive.leftFront.getCurrentPosition() - lLF;
                lLF = drive.leftFront.getCurrentPosition();

                dLB = drive.leftRear.getCurrentPosition() - lLB;
                lLB = drive.leftRear.getCurrentPosition();

                dRF = drive.rightFront.getCurrentPosition() - lRF;
                lRF = drive.rightFront.getCurrentPosition();

                dRB = drive.rightRear.getCurrentPosition() - lRB;
                lRB = drive.rightRear.getCurrentPosition();


                measure = t.milliseconds() + 100;
            }

            telemetry.addData("vals",String.format("%f %f %f %f",vals[0],vals[1],vals[2],vals[3]));
            telemetry.addData("dLF",dLF);
            telemetry.addData("dLB",dLB);
            telemetry.addData("dRF",dRF);
            telemetry.addData("dRB",dRB);
            telemetry.update();
        }


    }
}
