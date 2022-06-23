package com.example.thesis;

public class MatrixOperations {
    private static final int N = 2;

    //*********************************** matrx operation *******************************************//
    public static float[][] multiplyMatrices(float[][] firstMatrix, float[][] secondMatrix) {
        float[][] result = new float[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }
        return result;
    }

    public static float[][] addMatrixes(float[][] firstMatrix, float[][] secondMatrix) {
        float[][] result = new float[firstMatrix.length][firstMatrix.length];
        for (int i = 0; i < secondMatrix.length; i++) {
            for (int j = 0; j < secondMatrix.length; j++) {
                result[i][j] = firstMatrix[i][j] + secondMatrix[i][j];
            }
        }
        return result;
    }

    public static float[][] multiplyElementByElement(float[][] firstMatrix, float[][] secondMatrix) {
        float[][] result = new float[firstMatrix.length][firstMatrix.length];
        for (int i = 0; i < secondMatrix.length; i++) {
            for (int j = 0; j < secondMatrix.length; j++) {
                result[i][j] = firstMatrix[i][j] * secondMatrix[i][j];
            }
        }
        return result;
    }

    public static float[][] substractMatrices(float[][] firstMatrix, float[][] secondMatrix) {
        float[][] result = new float[firstMatrix.length][firstMatrix.length];
        for (int i = 0; i < secondMatrix.length; i++) {
            for (int j = 0; j < secondMatrix.length; j++) {
                result[i][j] = firstMatrix[i][j] - secondMatrix[i][j];
            }
        }
        return result;
    }

    public static float multiplyMatricesCell(float[][] firstMatrix, float[][] secondMatrix, int row, int col) {
        float cell = 0;
        for (int i = 0; i < secondMatrix.length; i++) {
            cell += firstMatrix[row][i] * secondMatrix[i][col];
        }
        return cell;
    }

    public static float[][] transposeMatrix(float[][] matrix) {
        int m = matrix.length;
        int n = matrix[0].length;

        float[][] transposedMatrix = new float[n][m];

        for (int x = 0; x < n; x++) {
            for (int y = 0; y < m; y++) {
                transposedMatrix[x][y] = matrix[y][x];
            }
        }

        return transposedMatrix;
    }

    static void getCofactor(float[][] A, float[][] temp, int p, int q, int n) {
        int i = 0, j = 0;


        for (int row = 0; row < n; row++) {
            for (int col = 0; col < n; col++) {
                if (row != p && col != q) {
                    temp[i][j++] = A[row][col];

                    if (j == n - 1) {
                        j = 0;
                        i++;
                    }
                }
            }
        }
    }

    static float determinant(float[][] A, int n) {
        int D = 0;
        if (n == 1)
            return A[0][0];
        float[][] temp = new float[N][N];
        int sign = 1;
        for (int f = 0; f < n; f++) {

            getCofactor(A, temp, 0, f, n);
            D += sign * A[0][f] * determinant(temp, n - 1);


            sign = -sign;
        }

        return D;
    }

    private static float[][] adjoint(float[][] A) {


        int sign = 1;
        float[][] temp = new float[N][N];
        final float[][] adj = new float[9][9];

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                getCofactor(A, temp, i, j, N);
                sign = ((i + j) % 2 == 0) ? 1 : -1;

                adj[j][i] = (sign) * (determinant(temp, N - 1));
            }
        }
        return adj;
    }


    public static float[][] inverse(float[][] A) {
        float[][] inverse = new float[2][2];
        float det = determinant(A, N);

        float[][] adj = adjoint(A);

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                inverse[i][j] = adj[i][j] / (float) det;

        return inverse;
    }

}

