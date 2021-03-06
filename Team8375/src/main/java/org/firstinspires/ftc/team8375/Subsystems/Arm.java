package org.firstinspires.ftc.team8375.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    // variable initialization
    public DcMotor lift;
    public DcMotor extend0;
    public DcMotor extend1;
    public CRServo intake;
    public Servo flip;
    float LiftPos;
    float highLimit;
    float lastLiftPos = 0;
    double intakePower = 0.5;

    public Arm(int targetStartPosition, HardwareMap hwMap) {
        lift = hwMap.dcMotor.get("lift");
        //left = 0, right = 1 (might not be correct but I'll test it later)
        extend0 = hwMap.dcMotor.get("extend_left");
        extend1 = hwMap.dcMotor.get("extend_right");
        intake = hwMap.crservo.get("intake");
        flip = hwMap.servo.get("flip");

        //motor initialization
        ArmMotorInit(targetStartPosition);
    }
// high limit - 2375
    public void setPowers(double liftPower, double extendPower, float intakeButton, boolean flipButton, float limitRange, float liftHigh, double autoGain) {

        if(intakeButton > 0){
            intake.setPower(-intakePower);
        } else{
            intake.setPower(intakePower);
        }
        if (flipButton) {
            flip.setPosition(180);
        } else {
            flip.setPosition(0);
        }

        //limits
        LiftPos = lift.getCurrentPosition();
        highLimit = -liftHigh + limitRange;
        if(liftPower > 0 && Math.abs(LiftPos) < limitRange){
            liftPower = -(LiftPos/limitRange);
            lastLiftPos = LiftPos;
        }
       else if (liftPower < 0 && LiftPos <= -highLimit) {
            liftPower = -(liftHigh + LiftPos)/limitRange;
            lastLiftPos = LiftPos;
        }
        //auto-correct function to make sure the arm is in the desired position when stopped.
        //sometimes the arm can sag under its own weight and this prevents that from happening
        else if(Math.abs(lastLiftPos-LiftPos)>10 && Math.abs(liftPower) < 0.175) {
            float error = lastLiftPos - LiftPos;
            liftPower = (error / autoGain);
        }

        //set powers
        lift.setPower(liftPower);
        extend0.setPower(extendPower);
        extend1.setPower(extendPower);
    }

    public void ArmMotorInit(int position) {

        //motor initialization
        lift.setDirection(DcMotor.Direction.FORWARD);
        extend0.setDirection(DcMotor.Direction.FORWARD);
        extend1.setDirection(DcMotor.Direction.REVERSE);
        //there's a lot but its all important
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setTargetPosition(-position);
        lift.setPower(0.05);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(!lift.isBusy()){
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extend0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extend1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
}