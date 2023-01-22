package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Reapr_HardwareClass {
    private static Reapr_HardwareClass robot = null;
    private HardwareMap hwMap;

    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;

    public Servo testServo = null;
    public static Reapr_HardwareClass getInstance(){
        if(robot == null){
            robot = new Reapr_HardwareClass();
        }
        return robot;

    }

    private void init(HardwareMap hardwareMap){
        hwMap = hardwareMap;

        //left wheels need to be reversed
        try{
            motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontLeft.setPower(0);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

        catch(Exception p_exception){
            motorFrontLeft = null;
        }

        try{
            testServo = hwMap.get(Servo.class, "testServo");
        }
        catch(Exception p_exception){
            testServo = null;
        }
    }

}
