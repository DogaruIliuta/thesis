package com.example.thesis;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Bundle;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;


import androidx.appcompat.app.AppCompatActivity;

import com.google.ar.sceneform.math.Quaternion;
import com.google.ar.sceneform.math.Vector3;

import java.util.Arrays;
import java.util.Locale;

public class SensorData extends AppCompatActivity implements SensorEventListener {


    private static final String TAG = "SensorData";

    final float alpha = 0.8f;

    private static final float NS2S = 1.0f / 1000000000.0f;
    private static final float EPSILON = (float) 1e-3;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp_gyro;
    private float timestamp_accel;
    private float timestamp_magnetometer;
    private final float[] mRotationMatrix = new float[16];
    float[][] measured_coordinatesFromAirdoc = new float[1][6];
    private boolean isAllowedToCompute = false;
    private boolean isFirstStep = true;
    private float[] acceleration = new float[3];
    private float[] displacement = new float[3];
    private float[] velocity = new float[3];
    float[] magnetometer = new float[3];
    float[] magneticValues = new float[3];
    float[] gravityValues = new float[3];
    float[] deltaRotationMatrix = new float[9];
    float[] next_vel = new float[3];


    private float instant_velovity;


    private SensorManager sensorManager;
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];
    private final float[] gravity = new float[3];
    private final float[] gravity_readings = new float[3];
    private final float[] linear_acceleration = new float[3];
    private final float[] linear_acceleration_test = new float[3];


    private final float[] rotationMatrix = new float[9];
    private final float[] orientationAngles = new float[3];
    private final float[][] systemState = new float[][]{new float[]{0}, new float[]{0}, new float[]{0}, new float[]{0}, new float[]{0}, new float[]{0}};
    private final KalmanPosition kalmanPosition = new KalmanPosition(systemState, 500, 2);





    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        displacement = new float[3];
        velocity = new float[3];
        setContentView(R.layout.activity_sensor_data);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        PackageManager packageManager = getPackageManager();
        boolean gyroExists = packageManager.hasSystemFeature(PackageManager.FEATURE_SENSOR_GYROSCOPE);
        Log.i(TAG, "Has gyro " + gyroExists);
        Button ad = findViewById(R.id.button2);
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        ad.setOnClickListener(v -> {
            if (ad.getText().toString().toUpperCase(Locale.ROOT).equals("TAKE MEASUREMENT")) {
                ad.setText("End Measurement");

                if (accelerometer != null) {
                    sensorManager.registerListener(this, accelerometer,
                            SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
                }
            }else {
                ad.setText("Take Measurement");
                if (accelerometer != null) {
                    sensorManager.unregisterListener(this, accelerometer);
                }
                isFirstStep = true;
                TextView x_text = (TextView) findViewById(R.id.textox);
                TextView y_text = (TextView) findViewById(R.id.textoy);
                TextView z_text = (TextView) findViewById(R.id.textoz);
                x_text.setText(String.valueOf(displacement[0]));
                y_text.setText(String.valueOf(displacement[1]));
                z_text.setText(String.valueOf(displacement[2]));
            }


        });


    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
        // You must implement this callback in your code.
    }

    @Override
    protected void onResume() {
        super.onResume();
        Sensor magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magneticField != null) {
            sensorManager.registerListener(this, magneticField,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
        Sensor rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        if (rotationVector != null) {
            sensorManager.registerListener(this, rotationVector,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
        Sensor gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if (gyroscope != null) {
            sensorManager.registerListener(this, gyroscope,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
        Sensor rotation = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        if (gyroscope != null) {
            sensorManager.registerListener(this, rotation,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }

        Sensor gravity = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        if (gyroscope != null) {
            sensorManager.registerListener(this, gravity,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
        Sensor linear = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        if (gyroscope != null) {
            sensorManager.registerListener(this, linear,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }


    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        TextView x_text = (TextView) findViewById(R.id.textox);
        TextView y_text = (TextView) findViewById(R.id.textoy);
        TextView z_text = (TextView) findViewById(R.id.textoz);
        if ((gravityValues != null) && (magneticValues != null)
                && (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)) {
            final float dT = (event.timestamp - timestamp_accel) * NS2S;
            Log.i(TAG, "dT "+ dT);

            float[] deviceRelativeAcceleration = new float[4];
            deviceRelativeAcceleration[0] = event.values[0];
            deviceRelativeAcceleration[1] = event.values[1];
            deviceRelativeAcceleration[2] = event.values[2];
            deviceRelativeAcceleration[3] = 0;

            // Change the device relative acceleration values to earth relative values
            // X axis -> East
            // Y axis -> North Pole
            // Z axis -> Sky

            float[] R = new float[16], I = new float[16], earthAcc = new float[16];

            SensorManager.getRotationMatrix(R, I, gravityValues, magneticValues);

            float[] inv = new float[16];

            android.opengl.Matrix.invertM(inv, 0, R, 0);
            android.opengl.Matrix.multiplyMV(earthAcc, 0, inv, 0, deviceRelativeAcceleration, 0);
            Log.d("Acceleration", "Values: (" + earthAcc[0] + ", " + earthAcc[1] + ", " + earthAcc[2] + ")");


            // Isolate the force of gravity with the low-pass filter.
            gravity[0] = alpha * gravity[0] + (1 - alpha) * earthAcc[0];
            gravity[1] = alpha * gravity[1] + (1 - alpha) * earthAcc[1];
            gravity[2] = alpha * gravity[2] + (1 - alpha) * earthAcc[2];

            // Remove the gravity contribution with the high-pass filter.
            linear_acceleration[0] = Math.abs(earthAcc[0] - gravity[2]) < 1e-2 ? 0 : earthAcc[0] - gravity[0];
            linear_acceleration[1] = Math.abs(earthAcc[1] - gravity[2]) < 1e-2 ? 0 : earthAcc[1] - gravity[1];
            linear_acceleration[2] = Math.abs(earthAcc[2] - gravity[2]) < 1e-2 ? 0 : earthAcc[2] - gravity[2];

            earthAcc[2] -= 9.717f;
            Log.i(TAG, "Linear gravity linear_acceleration "+ Arrays.toString(linear_acceleration));
            Log.i(TAG, "gravity linear_acceleration test "+ earthAcc[0] + " "+ earthAcc[1] + " "+ earthAcc[2]);
            Log.i(TAG, "gravity earth "+ Arrays.toString(earthAcc));


            // if the phone is steady
            if (isFirstStep && Math.abs(linear_acceleration[0]) < 1e-3 && Math.abs(linear_acceleration[1]) < 1e-3 && Math.abs(linear_acceleration[2]) < 1e-3) {
                Log.d(TAG, "first entry");
                velocity = new float[3];
                displacement = new float[3];
                isFirstStep = false;
            }
//            if (Math.abs(linear_acceleration[0]) < 1e-2 ){
//                linear_acceleration[0] = 0;
//
//            }
//            if (Math.abs(linear_acceleration[1]) < 1e-2 ){
//                linear_acceleration[1] = 0;
//
//            }
//            if (Math.abs(linear_acceleration[2]) < 1e-2 ){
//                linear_acceleration[2] = 0;
//
//            }

//            if (Math.abs(velocity[0]) < 5e-2  ){
//                velocity[0] = 0;
//            }
//            if (Math.abs(velocity[1]) < 5e-2  ){
//                velocity[1] = 0;
//            }
//            if (Math.abs(velocity[2]) < 5e-2 ){
//                velocity[2] = 0;
//            }

            Log.i(TAG, "velocity "+ Arrays.toString(velocity));
            Log.i(TAG, "velocity next "+ Arrays.toString(next_vel));


            displacement = new float[]{
                    displacement[0] + velocity[0] * dT + earthAcc[0] * dT * dT *0.5f,
                    displacement[1] + velocity[1] * dT + earthAcc[1] * dT * dT *0.5f,
                    displacement[2] + velocity[2] * dT + earthAcc[2] * dT * dT *0.5f
            };
            velocity = new float[]{
                    velocity[0] + earthAcc[0] * dT,
                    velocity[1] + earthAcc[1] * dT,
                    velocity[2] + earthAcc[2] * dT
            };
            if (Math.abs(velocity[2] - next_vel[2]) < 1e-2 &&
                    Math.abs(velocity[1] - next_vel[1]) < 1e-2 &&
                    Math.abs(velocity[0] - next_vel[0]) < 1e-2){
                Log.i(TAG, "velocity next in ");

                velocity = new float[3];
            }

            next_vel = velocity.clone();

            Log.d(TAG, "next displacement: " + Arrays.toString(displacement));
            //Log.d(TAG, "next velocity: " + Arrays.toString(velocity));
            Log.d(TAG, "magentometer: " + Arrays.toString(magnetometer));


            x_text.setText(String.valueOf(displacement[0]));
            y_text.setText(String.valueOf(displacement[1]));
            z_text.setText(String.valueOf(displacement[2]));

        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            gravityValues = event.values;
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            magneticValues = event.values;
        }
        timestamp_accel = event.timestamp;
    }

//    @Override
//    public void onSensorChanged(SensorEvent event) {
//
//        float[] accelerometer = new float[3];
//        float[] gyroscope = new float[3];
//
//
//        //Log.i(TAG, "Sensor type {}" + String.valueOf(event.sensor.getType()));
//        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
//
//
//            final float dT = (event.timestamp - timestamp_gyro) * NS2S;
//            Log.i(TAG, "timestamp_gyroscope :" + dT);
//            // Axis of the rotation sample, not normalized yet.
//            float axisX = event.values[0];
//            float axisY = event.values[1];
//            float axisZ = event.values[2];
//
//            gyroscope[0] = event.values[0];
//            gyroscope[1] = event.values[1];
//            gyroscope[2] = event.values[2];
//            //Log.i(TAG,"gyro_axisX :" + axisX);
//            //Log.i(TAG,"gyro_axisy :" + axisY);
//            //Log.i(TAG,"gyro_axisz :" + axisZ);
//            Log.i(TAG, "gyro :" + axisX + " " + axisY + " " + axisZ);
//
//            // Calculate the angular speed of the sample
//            float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
//            if (omegaMagnitude > EPSILON) {
//                axisX /= omegaMagnitude;
//                axisY /= omegaMagnitude;
//                axisZ /= omegaMagnitude;
//            }
//            float thetaOverTwo = omegaMagnitude * dT / 2.0f;
//            float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
//            float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
//            deltaRotationVector[0] = sinThetaOverTwo * axisX;
//            deltaRotationVector[1] = sinThetaOverTwo * axisY;
//            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
//            deltaRotationVector[3] = cosThetaOverTwo;
//
//            SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
//            timestamp_gyro = event.timestamp;
//
//
//        } else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
//            TextView x_text = (TextView) findViewById(R.id.textox);
//            TextView y_text = (TextView) findViewById(R.id.textoy);
//            TextView z_text = (TextView) findViewById(R.id.textoz);
//            acceleration = event.values.clone();
//
//            final float dT = (event.timestamp - timestamp_accel) * NS2S;
//
//            float[] deviceRelativeAcceleration = new float[4];
//            deviceRelativeAcceleration[0] = event.values[0];
//            deviceRelativeAcceleration[1] = event.values[1];
//            deviceRelativeAcceleration[2] = event.values[2];
//            deviceRelativeAcceleration[3] = 0;
//
//            // Change the device relative acceleration values to earth relative values
//            // X axis -> East
//            // Y axis -> North Pole
//            // Z axis -> Sky
//
//            float[] R = new float[16], I = new float[16], earthAcc = new float[16];
//            Log.d(TAG, "gravity " + Arrays.toString(gravityValues));
//
//            SensorManager.getRotationMatrix(R, I, gravityValues, magneticValues);
//
//            float[] inv = new float[16];
//
//            android.opengl.Matrix.invertM(inv, 0, R, 0);
//            android.opengl.Matrix.multiplyMV(earthAcc, 0, inv, 0, deviceRelativeAcceleration, 0);
//            Log.d("Acceleration", "Values: (" + earthAcc[0] + ", " + earthAcc[1] + ", " + earthAcc[2] + ")");
//
//
////            final float x_accel = event.values[0];
////            final float y_accel = event.values[1];
////            final float z_accel = event.values[2];
////            accelerometer[0] = x_accel;
////            accelerometer[1] = y_accel;
////            accelerometer[2] = z_accel;
////            accelerometerReading[0] = linear_acceleration[0];
////            accelerometerReading[1] = linear_acceleration[1];
////            accelerometerReading[2] = linear_acceleration[2];
////            updateOrientationAngles();
//
//
//            //float[] R = new float[9];
//            //float[] I = new float[9];
////            SensorManager.getRotationMatrix(R, I, event.values, magnetometer);
////            float [] A_D = event.values.clone();
////            float [] A_W = new float[3];
////            A_W[0] = R[0] * A_D[0] + R[1] * A_D[1] + R[2] * A_D[2];
////            A_W[1] = R[3] * A_D[0] + R[4] * A_D[1] + R[5] * A_D[2];
////            A_W[2] = R[6] * A_D[0] + R[7] * A_D[1] + R[8] * A_D[2];
//
//
//
//            // Isolate the force of gravity with the low-pass filter.
//            gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
//            gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
//            gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];
//            Log.i(TAG, "gravity "+ Arrays.toString(gravity));
//
//            // Remove the gravity contribution with the high-pass filter.
//            linear_acceleration[0] = Math.abs(event.values[2] - gravity[2]) < 1e-2 ? 0 : event.values[0] - gravity[0];
//            linear_acceleration[1] = Math.abs(event.values[2] - gravity[2]) < 1e-2 ? 0 : event.values[1] - gravity[1];
//            linear_acceleration[2] = Math.abs(event.values[2] - gravity[2]) < 1e-2 ? 0 : event.values[2] - gravity[2];
//
//            SensorManager.getRotationMatrix(R, I, event.values, magnetometer);
//
//
//
//            float [] A_D = linear_acceleration.clone();
//            float [] A_W = new float[3];
//            A_W[0] = R[0] * A_D[0] + R[1] * A_D[1] + R[2] * A_D[2];
//            A_W[1] = R[3] * A_D[0] + R[4] * A_D[1] + R[5] * A_D[2];
//            A_W[2] = R[6] * A_D[0] + R[7] * A_D[1] + R[8] * A_D[2];
//
//
//
//
//            Log.i(TAG, "test next displacement :" + (event.values[0] - gravity[0]) + " "+ (event.values[1] - gravity[1]) + " " + (event.values[2] - gravity[2]));
//            Log.i(TAG, "R 1 :" + Arrays.toString(deltaRotationMatrix));
//            Log.i(TAG, "R 2 :" + Arrays.toString(R));
//            Log.i(TAG, "linear test next displacement :" + linear_acceleration[0] + " "+ linear_acceleration[1] + " " + linear_acceleration[2]);
//
//
//            Log.i(TAG, "timestamp_accelerometer :" + dT);
//            Log.i(TAG, "accelerometer :" + event.values[0] + " " + event.values[1] + " " + event.values[2]);
//            Log.i(TAG, "linear_acceleration :" + linear_acceleration[0] + " " + linear_acceleration[1] + " " + linear_acceleration[2]);
//
//
//            if (isFirstStep && Math.abs(linear_acceleration[0]) < 1e-3 && Math.abs(linear_acceleration[1]) < 1e-3 && Math.abs(linear_acceleration[2]) < 1e-3) {
//                Log.d(TAG, "first entry");
//                velocity = new float[3];
//                displacement = new float[3];
//                isFirstStep = false;
//            }
//            if (Math.abs(linear_acceleration[0]) < 1e-3 ){
//                velocity[0] = 0;
//            }
//            if (Math.abs(linear_acceleration[1]) < 1e-3 ){
//                velocity[1] = 0;
//            }
//            if (Math.abs(linear_acceleration[2]) < 1e-3 ){
//                velocity[2] = 0;
//            }
////            if (Math.abs(velocity[0]) < 1e-3 ){
////                velocity[0] = 0;
////            }
////            if (Math.abs(velocity[1]) < 1e-3 ){
////                velocity[1] = 0;
////            }
////            if (Math.abs(velocity[2]) < 1e-3 ){
////                velocity[2] = 0;
////            }
//
//            displacement = new float[]{
//                    displacement[0] + velocity[0] * dT + A_W[0] * dT * dT *0.5f,
//                    displacement[1] + velocity[1] * dT + A_W[1] * dT * dT *0.5f,
//                    displacement[2] + velocity[2] * dT + A_W[2] * dT * dT *0.5f
//            };
//            //Log.d(TAG, "velocity O" + Arrays.toString(velocity));
//            Log.d(TAG, "AW O" + Arrays.toString(A_W));
//            Log.d(TAG, "velocity N " + (velocity[0] + A_W[0] * dT));
//            Log.d(TAG, "velocity E " + (velocity[1] + A_W[1] * dT));
//            Log.d(TAG, "velocity D " + (velocity[2] + A_W[2] * dT));
//            Log.d(TAG, "velocity L " + (linear_acceleration[2] * dT));
//
//            velocity = new float[]{
//                    velocity[0] + A_W[0] * dT,
//                    velocity[1] + A_W[1] * dT,
//                    velocity[2] + A_W[2] * dT
//            };
//            Log.d(TAG, "velocity O" + Arrays.toString(velocity));
//            x_text.setText(String.valueOf(displacement[0]));
//            y_text.setText(String.valueOf(displacement[1]));
//            z_text.setText(String.valueOf(displacement[2]));
//
//            Log.d(TAG, "rotation " + linear_acceleration[0] + " "+ linear_acceleration[1] + " " + linear_acceleration[2]);
//            //Log.d(TAG, "rotation magnetometer " + Arrays.toString(magnetometer));
//            //Log.d(TAG, "rotation acceleration " + Arrays.toString(event.values));
//
//            if (isFirstStep && linear_acceleration[0] == 0 && linear_acceleration[1] == 0 && linear_acceleration[2] ==0) {
//
//
//
//
//                Log.d(TAG, "is allowed to compute");
//                isAllowedToCompute = true;
//                isFirstStep = false;
//                velocity = new float[3];//{linear_acceleration[0] * dT, linear_acceleration[1] * dT, linear_acceleration[2] * dT};
//                //Log.d(TAG, "original velocity: " + Arrays.toString(velocity));
////
//                //displacement = new float[3];//{linear_acceleration[0] * dT * dT / 2, linear_acceleration[1] * dT * dT / 2, linear_acceleration[2] * dT * dT *0.5f};
//                //Log.d(TAG, "original displacement: " + Arrays.toString(displacement));
//
//            }
//
//            if (isAllowedToCompute) {
//                Log.d(TAG, "next displacements");
////                float[] R = new float[9];
////                float[] I = new float[9];
////                SensorManager.getRotationMatrix(R, I, event.values, magnetometer);
////                float [] A_D = event.values.clone();
////                float [] A_W = new float[3];
////                A_W[0] = R[0] * A_D[0] + R[1] * A_D[1] + R[2] * A_D[2];
////                A_W[1] = R[3] * A_D[0] + R[4] * A_D[1] + R[5] * A_D[2];
////                A_W[2] = R[6] * A_D[0] + R[7] * A_D[1] + R[8] * A_D[2];
//
//               // Log.d(TAG, "rotation " + Arrays.toString(R));
//
////                displacement = new float[]{
////                        displacement[0] + velocity[0] * dT + A_W[0] * dT * dT *0.5f,
////                        displacement[1] + velocity[1] * dT + A_W[1] * dT * dT *0.5f,
////                        displacement[2] + velocity[2] * dT + A_W[2] * dT * dT *0.5f
////                };
////                velocity = linear_acceleration[0] == 0 && linear_acceleration[1] == 0 && linear_acceleration[2] ==0 ? new float[3] : new float[]{
////                        velocity[0] + A_W[0] * dT,
////                        velocity[1] + A_W[1] * dT,
////                        velocity[2] + A_W[2] * dT
////                };
////
////                Log.d(TAG, "next displacement: " + Arrays.toString(displacement));
////                Log.d(TAG, "next velocity: " + Arrays.toString(velocity));
////                Log.d(TAG, "magentometer: " + Arrays.toString(magnetometer));
//
////                x_text.setText(String.valueOf(displacement[0]));
////                y_text.setText(String.valueOf(displacement[1]));
////                z_text.setText(String.valueOf(displacement[2]));
//
//
//            }
//
//
//            //Log.i(TAG, "axisX_ACCEL {}" + event.values[0]);
//            //Log.i(TAG, "axisy_ACCEL {}" + event.values[1]);
//            //Log.i(TAG, "axisz_ACCEL {}" + event.values[2]);
//
//
//            timestamp_accel = event.timestamp;
//
//
//        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
//            final float dT = (event.timestamp - timestamp_magnetometer) * NS2S;
//
//            Log.i(TAG, "timestamp_magnetometer :" + dT);
//
//            //Log.i(TAG, "magn_x {}" + event.values[0]);
//            //Log.i(TAG, "magn_y {}" + event.values[1]);
//            //Log.i(TAG, "magn_z {}" + event.values[2]);
//            magnetometer = event.values.clone();
//            Log.i(TAG, "magnetometer :" + magnetometer[0] + " " + magnetometer[1] + " " + magnetometer[2]);
//            Log.i(TAG, "magnetometer acceleration :" + accelerometer[0] + " " + accelerometer[1] + " " + accelerometer[2]);
//
//
//            //magnetometer[0] = event.values[0];
//            //magnetometer[1] = event.values[1];
//            //magnetometer[2] = event.values[2];
//            timestamp_magnetometer = event.timestamp;
//
//
//
//
//
//        } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR){
//             float[] rotationMatrix = new float[9];
//             float[] orientation = new float[3];
//            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values);
//            SensorManager.getOrientation(rotationMatrix, orientation);
//            Log.d(TAG, "onSensorChanged: YAW" + orientation[0]);
//            Log.d(TAG, "onSensorChanged: pitch" + orientation[1]);
//            Log.d(TAG, "onSensorChanged: roll" + orientation[2]);
//        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY){
//            gravityValues = event.values;
//        }else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
//            magneticValues = event.values;
//        }
//
//
//        //Kalman filter for orientation
//
//        final float dT_magn = (event.timestamp - timestamp_magnetometer) * NS2S;
//        final float dT_accel = (event.timestamp - timestamp_accel) * NS2S;
//        final float dT_gyro = (event.timestamp - timestamp_magnetometer) * NS2S;
//        // compose the N E D Axes
//        Vector3 magnetometer_unit = new Vector3(magnetometer[0], magnetometer[1], magnetometer[2]).normalized();
//        Vector3 accelerometer_unit = new Vector3(accelerometer[0], accelerometer[1], accelerometer[2]).normalized();
//
//        Vector3 D = accelerometer_unit.negated();
//        Vector3 E = Vector3.cross(D, magnetometer_unit).normalized();
//
//        Vector3 N = Vector3.cross(E, D).normalized();
//
//
//        // Build the DCM with the transpose of N E D
//        final float[][] DCM = new float[3][3];
//
//        DCM[0] = new float[]{N.x, E.x, D.x};
//        DCM[1] = new float[]{N.y, E.y, D.y};
//        DCM[2] = new float[]{N.z, E.z, D.z};
//
//        // DCm to quaternion
//
//        final float q_4 = (float) Math.sqrt(1.0 / 4 * (1 + DCM[0][0] + DCM[1][1] + DCM[2][2]));
//        final float q_3 = (float) Math.sqrt(1.0 / 4 * (1 - DCM[0][0] - DCM[1][1] + DCM[2][2]));
//        final float q_2 = (float) Math.sqrt(1.0 / 4 * (1 - DCM[0][0] + DCM[1][1] - DCM[2][2]));
//        final float q_1 = (float) Math.sqrt(1.0 / 4 * (1 + DCM[0][0] - DCM[1][1] - DCM[2][2]));
//
//        //Quaternion with resepct to NED axis
//
//        final Quaternion quaternionsFromDCM = new Quaternion(q_1, q_2, q_3, q_4);
//        Log.i("Quatenrions of is {}", quaternionsFromDCM.toString());
//
//        final float x_accel = accelerometer[0];
//        final float y_accel = accelerometer[1];
//        final float z_accel = accelerometer[2];
//        final float[][] accelInBodyFrame = new float[][]{new float[]{x_accel}, new float[]{y_accel}, new float[]{z_accel}};
//
//        //final float[][] accelerationOnNED = MatrixOperations.multiplyMatrices(MatrixOperations.inverse(DCM), accelInBodyFrame);
//        //final float N_Accel = accelerationOnNED[0][0];
//        //final float E_accel = accelerationOnNED[1][0];
//        //final float D_accel = accelerationOnNED[2][0];
//
//
//        //Kalman filter for position without taking into account acceleration
//        //kalmanPosition.predict(dT_accel);
//
//
//        // start with everything at 0
//        //kalmanPosition.update(measured_coordinatesFromAirdoc, dT_accel);
//
//
//    }

    public void computeDisplacement(final float dt, final float v, final float a) {

    }

    public void updateOrientationAngles() {
        // Update rotation matrix, which is needed to update orientation angles.
        SensorManager.getRotationMatrix(rotationMatrix, null,
                accelerometerReading, magnetometerReading);
        SensorManager.getOrientation(rotationMatrix, orientationAngles);
    }

}