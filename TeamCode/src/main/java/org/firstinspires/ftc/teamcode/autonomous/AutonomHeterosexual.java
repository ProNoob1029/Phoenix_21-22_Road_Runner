package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OmniSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "heterosexual")
@Config
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

        double measure = 0;
        while(isStopRequested() == false){
            double[] vals = OmniSimple.calculateAndSet(33/4,0,0);

            for (DcMotorEx motor :
                    drive.motors) {
                motor.setPower(1);
            }

            if(measure < t.milliseconds()){
                dLF = drive.leftFront.getCurrentPosition() - lLF;
                lLF = drive.leftFront.getCurrentPosition();

                dLB = drive.leftRear.getCurrentPosition() - lLB;
                lLB = drive.leftRear.getCurrentPosition();

                dRF = drive.rightFront.getCurrentPosition() - lRF;
                lRF = drive.rightFront.getCurrentPosition();

                dRB = drive.rightRear.getCurrentPosition() - lRB;
                lRB = drive.rightRear.getCurrentPosition();

                telemetry.addData("dLF",dLF);
                telemetry.addData("dLB",dLB);
                telemetry.addData("dRF",dRF);
                telemetry.addData("dRB",dRB);

                measure = t.milliseconds() + 100;
            }
        }


    }
}
