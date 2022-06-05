package com.example.thesis;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;


import androidx.appcompat.app.AppCompatActivity;

import com.google.ar.sceneform.math.Quaternion;
import com.google.ar.sceneform.math.Vector3;

public class SensorData extends AppCompatActivity implements SensorEventListener{

    private static final float NS2S = 1.0f / 1000000000.0f;
    private static final float EPSILON = (float) 1e-3;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp_gyro;
    private float timestamp_accel;
    private float timestamp_magnetometer;
    private final float[] mRotationMatrix = new float[16];



    private SensorManager sensorManager;
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];

    private final float[] rotationMatrix = new float[9];
    private final float[] orientationAngles = new float[3];
    private final float[][] systemState = new float[][] {new float[]{0}, new float[]{0}, new float[]{0}, new float[]{0}, new float[]{0}, new float[]{0}};
    private final KalmanPosition kalmanPosition = new KalmanPosition(systemState, 500, 2);

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor_data);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);


    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
        // You must implement this callback in your code.
    }

    @Override
    protected void onResume() {
        super.onResume();
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer != null) {
            sensorManager.registerListener(this, accelerometer,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
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
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        float [] accelerometer = new float[3];
        float [] gyroscope = new float[3];
        float [] magnetometer = new float[3];



        Log.i("Sensor type {}", String.valueOf(event.sensor.getType()));
        if (timestamp_gyro != 0) {
            if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {


                final float dT = (event.timestamp - timestamp_gyro) * NS2S;
                // Axis of the rotation sample, not normalized yet.
                float axisX = event.values[0];
                float axisY = event.values[1];
                float axisZ = event.values[2];

                gyroscope[0] = event.values[0];
                gyroscope[1] = event.values[1];
                gyroscope[2] = event.values[2];
                Log.i("gyro_axisX {}", String.valueOf(axisX));
                Log.i("gyro_axisy {}", String.valueOf(axisY));
                Log.i("gyro_axisz {}", String.valueOf(axisZ));

                // Calculate the angular speed of the sample
                float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
                if (omegaMagnitude > EPSILON) {
                    axisX /= omegaMagnitude;
                    axisY /= omegaMagnitude;
                    axisZ /= omegaMagnitude;
                }
                float thetaOverTwo = omegaMagnitude * dT / 2.0f;
                float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
                float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
                deltaRotationVector[0] = sinThetaOverTwo * axisX;
                deltaRotationVector[1] = sinThetaOverTwo * axisY;
                deltaRotationVector[2] = sinThetaOverTwo * axisZ;
                deltaRotationVector[3] = cosThetaOverTwo;
                float[] deltaRotationMatrix = new float[9];
                SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
                timestamp_gyro = event.timestamp;
            }



        }else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            final float dT = (event.timestamp - timestamp_accel) * NS2S;
            final float x_accel = event.values[0];
            final float y_accel = event.values[1];
            final float z_accel = event.values[2];
            accelerometer[0] = x_accel;
            accelerometer[1] = y_accel;
            accelerometer[2] = z_accel;

            Log.i("axisX_ACCEL {}", String.valueOf(event.values[0]));
            Log.i("axisy_ACCEL {}", String.valueOf(event.values[1]));
            Log.i("axisz_ACCEL {}", String.valueOf(event.values[2]));
            timestamp_accel = event.timestamp;

        }else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
            final float dT = (event.timestamp - timestamp_magnetometer) * NS2S;
            Log.i("magn_x {}", String.valueOf(event.values[0]));
            Log.i("magn_y {}", String.valueOf(event.values[1]));
            Log.i("magn_z {}", String.valueOf(event.values[2]));

            magnetometer[0] = event.values[0];
            magnetometer[1] = event.values[1];
            magnetometer[2] = event.values[2];
            timestamp_magnetometer = event.timestamp;

        }

        //Kalman filter for orientation


        // compuse the N E D Axes
        Vector3 magnetometer_unit = new Vector3(magnetometer[0], magnetometer[1], magnetometer[2]).normalized();
        Vector3 accelerometer_unit = new Vector3(accelerometer[0], accelerometer[1], accelerometer[2]).normalized();

        Vector3 D = accelerometer_unit.negated();
        Vector3 E = Vector3.cross(D, magnetometer_unit).normalized();

        Vector3 N = Vector3.cross(E, D).normalized();


        // Build the DCM with the transpose of N E D
        final float[][] DCM = new float[3][3];

        DCM[0] = new float[]{N.x, E.x, D.x};
        DCM[1] = new float[]{N.y, E.y, D.y};
        DCM[2] = new float[]{N.z, E.z, D.z};

        // DCm to quaternion

        final float q_4 = (float) Math.sqrt(1.0 / 4 * ( 1 + DCM[0][0] + DCM[1][1] + DCM[2][2]));
        final float q_3 = (float) Math.sqrt(1.0 / 4 * ( 1 - DCM[0][0] - DCM[1][1] + DCM[2][2]));
        final float q_2 = (float) Math.sqrt(1.0 / 4 * ( 1 - DCM[0][0] + DCM[1][1] - DCM[2][2]));
        final float q_1 = (float) Math.sqrt(1.0 / 4 * ( 1 + DCM[0][0] - DCM[1][1] - DCM[2][2]));

        //Quaternion with resepct to NED axis

        final Quaternion quaternionsFromDCM = new Quaternion(q_1, q_2, q_3, q_4);
        Log.i("Quatenrions of is {}", quaternionsFromDCM.toString());

        final float x_accel = accelerometer[0];
        final float y_accel = accelerometer[1];
        final float z_accel = accelerometer[2];
        final float[][] accelInBodyFrame = new float[][]{new float[]{x_accel}, new float[]{y_accel}, new float[]{z_accel}};

        final float[][] accelerationOnNED = MatrixOperations.multiplyMatrices(MatrixOperations.inverse(DCM), accelInBodyFrame);
        final float N_Accel = accelerationOnNED[0][0];
        final float E_accel = accelerationOnNED[1][0];
        final float D_accel = accelerationOnNED[2][0];


        //Kalman filter for position without taking into account acceleration
        kalmanPosition.predict(timestamp_accel);
        float[][] measured_coordinatesFromAirdoc = new float[0][0];
        kalmanPosition.update(measured_coordinatesFromAirdoc, timestamp_accel);


    }

    public void updateOrientationAngles() {
        // Update rotation matrix, which is needed to update orientation angles.
        SensorManager.getRotationMatrix(rotationMatrix, null,
                accelerometerReading, magnetometerReading);
        SensorManager.getOrientation(rotationMatrix, orientationAngles);
    }

}