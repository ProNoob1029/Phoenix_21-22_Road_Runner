package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartLift {

    public DcMotorEx lift;

    public int pos0 = 0;
    public int pos1 = 0;
    public int pos2 = 0;
    public int pos3 = 0;

    public SmartLift(HardwareMap hardwareMap, String liftName){
        lift = hardwareMap.get(DcMotorEx.class, liftName);

        lift.setTargetPosition(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
