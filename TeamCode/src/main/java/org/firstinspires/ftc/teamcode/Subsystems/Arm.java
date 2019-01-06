package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotor lift;
    public DcMotor[] extend;
    int LiftPos;


    public Arm(HardwareMap hardwareMap) {
        lift = hardwareMap.dcMotor.get("lift");
        //left = 0, right = 1
        extend[0] = hardwareMap.dcMotor.get("extend_left");
        extend[1] = hardwareMap.dcMotor.get("extend_right");

        //motor initialization
        lift.setDirection(DcMotor.Direction.FORWARD);
        extend[0].setDirection(DcMotor.Direction.FORWARD);
        extend[1].setDirection(DcMotor.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void setPowers(double liftPower, double extendPower, int liftLow) {
        //limits
        LiftPos = lift.getCurrentPosition();
        if(liftPower > 0 && LiftPos < liftLow){
            liftPower = -(LiftPos/liftLow);
        }
       /* if (liftPower < 0 && LiftPos > liftHigh) {
            liftPower = LiftPos/liftHigh;
        }*/

        lift.setPower(liftPower);
        extend[0].setPower(extendPower);
        extend[1].setPower(extendPower);

    }
}
