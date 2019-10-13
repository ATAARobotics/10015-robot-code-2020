/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static java.lang.Math.abs;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front Left  drive motor:        "fl_drive"  Tetrix  1:60 Port 0
 * Motor channel:  Front Right drive motor:        "fr_drive" Tetrix  1:60 Port 1
 * Motor channel:  Back Left drive motor:        "bl_drive" Tetrix 1:60 Port 2
 * Motor channel:  Back Right drive motor:        "br_drive" Tetrix 1:60 Port 3
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 */
public class SwatRedBot_no_vf
{
    /* Public OpMode members. */
    public DcMotor  fl_Drive   = null;
    public DcMotor  fr_Drive  = null;
    public DcMotor  bl_Drive   = null;
    public DcMotor  br_Drive  = null;


    //Servos
    public Servo    clamp   = null;
    public Servo    left_gripper  = null;
    public Servo    right_gripper  = null;

    //Distance sensors
    public Rev2mDistanceSensor dis_rear; //2M distance sensor front
    public Rev2mDistanceSensor dis_front_left; //2M distance sensor rear left
    public Rev2mDistanceSensor dis_front_right; //2M distance sensor rear right

    //Color sensors
    public NormalizedColorSensor colorSensor;    // Hardware Device Object

    //Limit switch
    public DigitalChannel limit_1;

    //Drive options
    public enum DRIVE_OPTION {
        STRAIGHT,
        TURN
    }


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    // Motors' parameters, wheels as well
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.3;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     SPOOL_DIAMETER_INCHES   = 0.5;      //diameter of spool shaft

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH_HEIGHT         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (SPOOL_DIAMETER_INCHES * 3.1415);
    static boolean          is_gyroready            = false;
    static double  MIN_POSITION = 0, MAX_POSITION = 1; //define servos' Max and Min limited position

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();





    /* Constructor */
    public SwatRedBot_no_vf(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //Drive motors
        fl_Drive  = hwMap.get(DcMotor.class, "fl_drive");
        fr_Drive = hwMap.get(DcMotor.class, "fr_drive");
        bl_Drive  = hwMap.get(DcMotor.class, "bl_drive");
        br_Drive = hwMap.get(DcMotor.class, "br_drive");
        //Servos
        clamp = hwMap.get(Servo.class,"clamp");
        left_gripper = hwMap.get(Servo.class,"left_gripper");
        right_gripper = hwMap.get(Servo.class,"right_gripper");
        //Distance sensors
        dis_rear = hwMap.get(Rev2mDistanceSensor.class, "dis_rear");
        dis_front_left = hwMap.get(Rev2mDistanceSensor.class, "dis_front_left");
        dis_front_right = hwMap.get(Rev2mDistanceSensor.class, "dis_front_right");
        //Gyro sensor
        imu = hwMap.get(BNO055IMU.class, "imu");
        // color sensors
        colorSensor = hwMap.get(NormalizedColorSensor .class, "sensor_color");
        //Limit Switch
        limit_1 = hwMap.get(DigitalChannel.class,"limit_1");
        limit_1.setMode(DigitalChannel.Mode.INPUT);

        init_Gyro();   // initial gyro sensor

        fl_Drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        bl_Drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        fr_Drive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        br_Drive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //Servos direction
        left_gripper.setDirection(Servo.Direction.FORWARD);
        right_gripper.setDirection(Servo.Direction.REVERSE);
        left_gripper.setPosition(MIN_POSITION);
        right_gripper.setPosition(MAX_POSITION);

        // Set all motors to zero power
        fl_Drive.setPower(0);
        bl_Drive.setPower(0);
        fr_Drive.setPower(0);
        fr_Drive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fl_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }
    /**
     * Initialize the Gyrosensor wait until it is initialized
     */
    public void init_Gyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated())
        {
            is_gyroready = false;
        }
        is_gyroready = true;
    }

    /**
     *Gripper operation
     */
    public void grip(String motion){
        if (motion == "open"){
            left_gripper.setPosition(MIN_POSITION);
            right_gripper.setPosition(MIN_POSITION);


        } else if (motion == "close"){
            left_gripper.setPosition(MAX_POSITION);
            right_gripper.setPosition(MAX_POSITION);
        }
    }

    /**
     * Strafe left or right
     * @param power
     * @param direction
     */
    public void strafe(double power,String direction){
        if(direction =="left"){
            fl_Drive.setPower(-power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(-power);
        }
        if(direction == "right"){
            fl_Drive.setPower(power);
            fr_Drive.setPower(-power);
            bl_Drive.setPower(-power);
            br_Drive.setPower(power);
        }
    }



    /**
     * Use encoder to straff left or right
     * @param power
     * @param distance    unit inch
     * @param direction  "left" or "right"
     */

    public void encoder_Drive(double power, double distance,String direction)
    {
        int newFrontLeftTarget = 0;
        int newBackLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackRightTarget = 0;

        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction == "right"){
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = fl_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newBackLeftTarget = bl_Drive.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = fr_Drive.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newBackRightTarget = br_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        }
        if (direction == "left"){
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = fl_Drive.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newBackLeftTarget = bl_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = fr_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newBackRightTarget = br_Drive.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        }
        if (direction == "normal"){
        newFrontLeftTarget = fl_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newBackLeftTarget = bl_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newFrontRightTarget = fr_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newBackRightTarget = br_Drive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
    }


        fl_Drive.setTargetPosition(newFrontLeftTarget);
        bl_Drive.setTargetPosition(newBackLeftTarget);
        fr_Drive.setTargetPosition(newFrontRightTarget);
        br_Drive.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        fl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.

        fl_Drive.setPower(abs(power));
        fr_Drive.setPower(abs(power));
        bl_Drive.setPower(abs(power));
        br_Drive.setPower(abs(power));
        while (fl_Drive.isBusy() && fr_Drive.isBusy()&& br_Drive.isBusy()&& bl_Drive.isBusy()) {
        
        }

        // Stop all motion;
        stop();
        // Turn off RUN_TO_POSITION
        fl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop(){
        // Stop all motion;
        fl_Drive.setPower(0);
        bl_Drive.setPower(0);
        fr_Drive.setPower(0);
        br_Drive.setPower(0);
    }
    /**
    public double read_color(){
        // Read the sensor
        colorSensor.enableLed(true); // Turn the LED on
        colorSensor.enableLed(false); // Turn the LED off
        return colorSensor.blue(); // Combined color value

    }
     */
    /**
     * PID rotate
     */



    /**
     * Gyro Reading function
     */
    public Orientation GyroReading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }

    /**
     *
     * @param option STRAIGHT,TURN
     * @param power BASED ON JOYSTICK VALUE
     */
    public void drive(DRIVE_OPTION option,double power){
        switch (option){
            case STRAIGHT:
                fl_Drive.setPower(power);
                bl_Drive.setPower(power);
                fr_Drive.setPower(power);
                br_Drive.setPower(power);
                break;
            case TURN:
                fl_Drive.setPower(-power);
                bl_Drive.setPower(-power);
                fr_Drive.setPower(power);
                br_Drive.setPower(power);
                break;
        }
    }



}

