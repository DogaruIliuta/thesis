package com.example.thesis;

import java.util.Arrays;

public class KalmanPosition {

    private float[][] systemState = new float [1][6];
    private float[][] estimatedCurrentSystemState = new float [1][6];
    private final float[][] H = {new float[]{1, 0, 0, 0, 0, 0}, new float[]{0, 0, 0, 1, 0, 0}};
    private final float positionError;
    private float uncertainty;
    private final float[][] R = {new float[]{uncertainty, 0}, new float[]{0,uncertainty}};
    private final float[][] I_9 = {
        new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0}
    };

    private float[][] P;
    private float[][] K;



    public KalmanPosition(float[][] systemState, float uncertainty, float positionError){
        this.systemState = systemState;
        this.uncertainty = uncertainty;
        //value to be tuned
        this.positionError = positionError;
        this.P = getPInitial();
        this.K = new float[0][0];
    }


    //processUncertainty
    public void setNextP(final float dT){
        final float[][] F = getF(dT);
        final float[][] Q = getQ(dT, 1);
        final float [][] firstTerm = MatrixOperations.multiplyMatrices(MatrixOperations.multiplyMatrices(F, P), MatrixOperations.transposeMatrix(F));

        this.P = MatrixOperations.addMatrixes(firstTerm, Q);
    }
    public float[][] getPInitial(){
        return new float[][]{
                new float[]{this.positionError, 0, 0, 0, 0, 0},
                new float[]{0, this.positionError, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, this.positionError, 0, 0},
                new float[]{0, 0, 0, 0, this.positionError, 0},
                new float[]{0, 0, 0, 0, 0, 0}
        };
    }

    public void updateP(final float dT){
        final float[][] F = getF(dT);
        final float[][] Q = getQ(dT, 1);
        final float [][] firstTerm = MatrixOperations.substractMatrices(I_9, MatrixOperations.multiplyMatrices(K, H));
        final float [][] firstFullTerm = MatrixOperations.multiplyMatrices(MatrixOperations.multiplyMatrices(firstTerm, P), MatrixOperations.transposeMatrix(firstTerm));
        final float [][] secondFullTerm = MatrixOperations.multiplyMatrices(MatrixOperations.multiplyMatrices(K, R), MatrixOperations.transposeMatrix(K));
        this.P = MatrixOperations.addMatrixes(firstFullTerm, secondFullTerm);
    }



    private float[][] getF(final float dT){
        return new float[][]{
                new float[]{1, dT, dT * dT/2, 0, 0, 0},
                new float[]{0, 1, dT, 0, 0, 0},
                new float[]{0, 0, 1, 0, 0, 0},
                new float[]{0, 0, 0, 1, dT, dT * dT/2},
                new float[]{0, 0, 0, 0, 1, dT},
                new float[]{0, 0, 0, 0, 0, 1}
        };
    }



    public void predictNextState(final float dT){
        // the model assumes no acceleration
        this.systemState = MatrixOperations.multiplyMatrices(getF(dT), this.systemState);
    }
    public void estimateCurrentState(final float[][] measured){
        // the model assumes no acceleration
        final float[][] secondTerm = MatrixOperations.multiplyMatrices(K, MatrixOperations.substractMatrices(measured, MatrixOperations.multiplyMatrices(H, this.systemState)));
        this.estimatedCurrentSystemState = MatrixOperations.addMatrixes(this.systemState, secondTerm);
    }

///***********************************************************************************************//

    ///////////////Process Noise matrix//////////////

    public float[][] getQ(final float dt, final float rv){
        return new float[][]{
                new float[]{(float) (Math.pow(dt, 4)/4) * rv, (float) (Math.pow(dt, 3)/2) * rv, (float) (Math.pow(dt, 2)/2) * rv, 0, 0, 0},
                new float[]{(float) (Math.pow(dt, 3)/2) * rv, dt*dt * rv, dt*rv, 0, 0, 0},
                new float[]{(float) (Math.pow(dt, 2)/2) * rv, dt*rv, 1*rv, 0, 0, 0},
                new float[]{0, 0, 0, (float) (Math.pow(dt, 4)/4) * rv, (float) (Math.pow(dt, 3)/2) * rv, (float) (Math.pow(dt, 2)/2)},
                new float[]{0, 0, 0, (float) (Math.pow(dt, 3)/2) * rv, dt*dt*rv, dt*rv},
                new float[]{0, 0, 0, (float) (Math.pow(dt, 2)/2) * rv, dt*rv, 1*rv}
        };
    }

    //// Kalman gain
    public float[][] getKalmanGain(){
        final float[][] secondTerm = MatrixOperations.addMatrixes(MatrixOperations.multiplyMatrices(MatrixOperations.multiplyMatrices(H, P), MatrixOperations.transposeMatrix(H)), R);
        return MatrixOperations.multiplyMatrices(MatrixOperations.multiplyMatrices(P, MatrixOperations.transposeMatrix(H)), MatrixOperations.inverse(secondTerm));
    }

    public void predict(final float dt){
        predictNextState(dt);
        setNextP(dt);
    }

    public void update(final float[][] measured, final float dt){
        updateKalmanGain();
        estimateCurrentState(measured);
        updateP(dt);
    }

    public void updateKalmanGain(){
        this.K = getKalmanGain();
    }
}
