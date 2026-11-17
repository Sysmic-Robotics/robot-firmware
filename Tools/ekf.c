#include "ekf.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Helper: convert upper-triangle packed P to full 7×7 matrix ── */
/* Storage is sequential: row 0 has N elements, row 1 has N-1, etc. */
/* Diagonal index for row i = i*N - i*(i-1)/2, but we just iterate sequentially */
static void ekf_unpack_P(const float *P, float Pfull[EKF_N][EKF_N])
{
    memset(Pfull, 0, sizeof(float) * EKF_N * EKF_N);
    uint8_t idx = 0;
    for (uint8_t i = 0; i < EKF_N; i++) {
        Pfull[i][i] = P[idx++]; /* diagonal */
        for (uint8_t j = i + 1; j < EKF_N; j++) {
            float val = P[idx++];
            Pfull[i][j] = val;
            Pfull[j][i] = val; /* symmetrize */
        }
    }
}

/* ── Helper: pack full 7×7 matrix back to upper-triangle P ── */
static void ekf_pack_P(float *P, const float Pfull[EKF_N][EKF_N])
{
    uint8_t idx = 0;
    for (uint8_t i = 0; i < EKF_N; i++) {
        P[idx++] = Pfull[i][i]; /* diagonal */
        for (uint8_t j = i + 1; j < EKF_N; j++) {
            P[idx++] = Pfull[i][j]; /* upper triangle */
        }
    }
}

/* ── Analytical 3×3 inverse via adjugate/determinant ── */
/* Returns 0 on success, -1 if singular */
static int8_t inv3(const float A[3][3], float Ainv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < 1e-12f) return -1;

    float inv_det = 1.0f / det;

    Ainv[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    Ainv[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * inv_det;
    Ainv[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    Ainv[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * inv_det;
    Ainv[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    Ainv[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * inv_det;
    Ainv[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    Ainv[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * inv_det;
    Ainv[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

    return 0;
}

/* ── Gauss-Jordan inverse for NxN matrix ── */
/* Works in-place on augmented [A|I] — max N=5 for engine update */
#define GJ_MAX 5
static int8_t inv_gauss_jordan(const float *A, float *Ainv, uint8_t n)
{
    float aug[GJ_MAX][2 * GJ_MAX];
    memset(aug, 0, sizeof(aug));

    /* Build augmented matrix [A | I] */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            aug[i][j] = A[i * n + j];
            aug[i][n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    /* Forward elimination with partial pivoting */
    for (uint8_t col = 0; col < n; col++) {
        /* Find pivot */
        float max_val = fabsf(aug[col][col]);
        uint8_t max_row = col;
        for (uint8_t row = col + 1; row < n; row++) {
            if (fabsf(aug[row][col]) > max_val) {
                max_val = fabsf(aug[row][col]);
                max_row = row;
            }
        }
        if (max_val < 1e-12f) return -1;

        /* Swap rows */
        if (max_row != col) {
            for (uint8_t j = 0; j < 2 * n; j++) {
                float tmp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }

        /* Scale pivot row */
        float pivot = aug[col][col];
        float inv_pivot = 1.0f / pivot;
        for (uint8_t j = 0; j < 2 * n; j++) {
            aug[col][j] *= inv_pivot;
        }

        /* Eliminate column in all other rows */
        for (uint8_t row = 0; row < n; row++) {
            if (row == col) continue;
            float factor = aug[row][col];
            for (uint8_t j = 0; j < 2 * n; j++) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    /* Extract inverse from right half */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            Ainv[i * n + j] = aug[i][n + j];
        }
    }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════ */
/*                           EKF_Init                               */
/* ══════════════════════════════════════════════════════════════════ */
void EKF_Init(EKF_t *ekf, float dt)
{
    memset(ekf, 0, sizeof(EKF_t));

    ekf->dt = dt;

    /* Initial state: pointing in +x (θ=0, sinθ=0, cosθ=1) */
    ekf->x[2] = 0.0f;  /* sin(0) */
    ekf->x[3] = 1.0f;  /* cos(0) */

    /* Initial covariance — generous initial uncertainty */
    /* Sequential layout: row 0 → 7 elem, row 1 → 6, ..., row 6 → 1 elem */
    /* Diagonal indices: 0, 7, 13, 18, 22, 25, 27 */
    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[0]  = 0.5f;   /* P[0][0] = x variance */
    ekf->P[7]  = 0.5f;   /* P[1][1] = y variance */
    ekf->P[13] = 0.1f;   /* P[2][2] = sinθ variance */
    ekf->P[18] = 0.1f;   /* P[3][3] = cosθ variance */
    ekf->P[22] = 0.1f;   /* P[4][4] = vx variance */
    ekf->P[25] = 0.1f;   /* P[5][5] = vy variance */
    ekf->P[27] = 0.05f;  /* P[6][6] = ω variance */

    /* Process noise diagonal — tuned for 200Hz */
    ekf->Q[0] = 0.01f;   /* x */
    ekf->Q[1] = 0.01f;   /* y */
    ekf->Q[2] = 0.001f;  /* sinθ */
    ekf->Q[3] = 0.001f;  /* cosθ */
    ekf->Q[4] = 0.5f;    /* vx — accelerometer drives this */
    ekf->Q[5] = 0.5f;    /* vy — accelerometer drives this */
    ekf->Q[6] = 0.1f;    /* ω  — gyroscope drives this */

    /* Encoder measurement noise */
    ekf->R_enc[0] = 0.05f;  /* vx encoder */
    ekf->R_enc[1] = 0.05f;  /* vy encoder */
    ekf->R_enc[2] = 0.02f;  /* ω encoder */

    /* Engine (camera/external) measurement noise */
    ekf->R_eng[0] = 0.1f;   /* x position */
    ekf->R_eng[1] = 0.1f;   /* y position */
    ekf->R_eng[2] = 0.3f;   /* vx world */
    ekf->R_eng[3] = 0.3f;   /* vy world */
    ekf->R_eng[4] = 0.05f;  /* θ */

    /* History */
    ekf->hist_idx = 0;
    ekf->hist_count = 0;
    ekf->time = 0.0f;
}

/* ══════════════════════════════════════════════════════════════════ */
/*                          EKF_Predict                             */
/* ══════════════════════════════════════════════════════════════════ */
void EKF_Predict(EKF_t *ekf, float ax_body, float ay_body, float gyro_z)
{
    static float Pfull[EKF_N][EKF_N];
    static float F[EKF_N][EKF_N];
    static float tmp[EKF_N][EKF_N]; /* F · Pfull */

    /* Unpack upper-triangle P into full matrix */
    ekf_unpack_P(ekf->P, Pfull);

    float dt = ekf->dt;
    float s  = ekf->x[2];  /* sinθ */
    float c  = ekf->x[3];  /* cosθ */
    float vx = ekf->x[4];  /* vx robot frame */
    float vy = ekf->x[5];  /* vy robot frame */
    float wz = ekf->x[6];  /* ω */

    float theta    = atan2f(s, c);
    float theta_new = theta + wz * dt;
    float s_new    = sinf(theta_new);
    float c_new    = cosf(theta_new);

    /* ── State prediction ── */
    float x_new[EKF_N];
    x_new[0] = ekf->x[0] + (vx * c - vy * s) * dt;
    x_new[1] = ekf->x[1] + (vx * s + vy * c) * dt;
    x_new[2] = s_new;
    x_new[3] = c_new;
    x_new[4] = vx + ax_body * dt;
    x_new[5] = vy + ay_body * dt;
    x_new[6] = gyro_z;

    /* ── Jacobian F (start from identity, add off-diagonal) ── */
    memset(F, 0, sizeof(F));
    for (uint8_t i = 0; i < EKF_N; i++) F[i][i] = 1.0f;

    F[0][2] = -vy * dt;
    F[0][3] =  vx * dt;
    F[0][4] =  c * dt;
    F[0][5] = -s * dt;

    F[1][2] =  vx * dt;
    F[1][3] =  vy * dt;
    F[1][4] =  s * dt;
    F[1][5] =  c * dt;

    F[2][6] = dt * c_new;
    F[3][6] = -dt * s_new;

    /* ── Covariance: Pfull = F · Pfull · F^T + Q ── */
    /* tmp = F · Pfull */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = 0; j < EKF_N; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_N; k++) {
                sum += F[i][k] * Pfull[k][j];
            }
            tmp[i][j] = sum;
        }
    }

    /* Pfull = tmp · F^T (= tmp · F because F^T[j][i] = F[i][j]) */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = 0; j < EKF_N; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_N; k++) {
                sum += tmp[i][k] * F[j][k]; /* F^T[k][j] = F[j][k] */
            }
            Pfull[i][j] = sum;
        }
    }

    /* Add process noise Q to diagonal */
    for (uint8_t i = 0; i < EKF_N; i++) {
        Pfull[i][i] += ekf->Q[i];
    }

    /* ── Store results ── */
    memcpy(ekf->x, x_new, sizeof(float) * EKF_N);
    ekf_pack_P(ekf->P, Pfull);
    ekf->time += dt;
}

/* ══════════════════════════════════════════════════════════════════ */
/*                      EKF_UpdateEncoders                           */
/* ══════════════════════════════════════════════════════════════════ */
void EKF_UpdateEncoders(EKF_t *ekf, float vx_enc, float vy_enc, float vth_enc)
{
    static float Pfull[EKF_N][EKF_N];
    static float S[3][3], S_inv[3][3];
    static float K[EKF_N][3];

    ekf_unpack_P(ekf->P, Pfull);

    /* ── Innovation: z - h(x) ── */
    float y[3];
    y[0] = vx_enc - ekf->x[4];
    y[1] = vy_enc - ekf->x[5];
    y[2] = vth_enc - ekf->x[6];

    /* ── S = H·P·H^T + R (H picks rows 4,5,6 — exploit sparsity) ── */
    S[0][0] = Pfull[4][4] + ekf->R_enc[0];
    S[0][1] = Pfull[4][5];
    S[0][2] = Pfull[4][6];
    S[1][0] = Pfull[5][4];
    S[1][1] = Pfull[5][5] + ekf->R_enc[1];
    S[1][2] = Pfull[5][6];
    S[2][0] = Pfull[6][4];
    S[2][1] = Pfull[6][5];
    S[2][2] = Pfull[6][6] + ekf->R_enc[2];

    /* ── Inverse S (analytical 3×3) ── */
    if (inv3(S, S_inv) != 0) return; /* skip update if singular */

    /* ── Kalman gain: K = P · H^T · S^-1 ── */
    /* H^T columns correspond to states 4,5,6, so K[i][j] = Pfull[i][j+4] · S_inv */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 3; k++) {
                sum += Pfull[i][k + 4] * S_inv[k][j];
            }
            K[i][j] = sum;
        }
    }

    /* ── State update: x += K · y ── */
    for (uint8_t i = 0; i < EKF_N; i++) {
        ekf->x[i] += K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
    }

    /* ── Renormalize sinθ, cosθ to unit circle ── */
    {
        float s = ekf->x[2];
        float c = ekf->x[3];
        float norm = sqrtf(s * s + c * c);
        if (norm > 1e-6f) {
            ekf->x[2] = s / norm;
            ekf->x[3] = c / norm;
        }
    }

    /* ── Covariance update: P = (I - K·H) · P ── */
    /* KH[i][j] = K[i][j-4] if j >= 4, else 0 */
    /* P_new[i][j] = P[i][j] - K[i][0]*P[4][j] - K[i][1]*P[5][j] - K[i][2]*P[6][j] */
    /* Save rows 4,5,6 before modifying — they are read by all rows including themselves */
    float P4[EKF_N], P5[EKF_N], P6[EKF_N];
    for (uint8_t j = 0; j < EKF_N; j++) {
        P4[j] = Pfull[4][j];
        P5[j] = Pfull[5][j];
        P6[j] = Pfull[6][j];
    }
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = 0; j < EKF_N; j++) {
            Pfull[i][j] -= K[i][0] * P4[j]
                          + K[i][1] * P5[j]
                          + K[i][2] * P6[j];
        }
    }

    /* ── Enforce symmetry ── */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = i + 1; j < EKF_N; j++) {
            float avg = 0.5f * (Pfull[i][j] + Pfull[j][i]);
            Pfull[i][j] = avg;
            Pfull[j][i] = avg;
        }
    }

    ekf_pack_P(ekf->P, Pfull);
}

/* ══════════════════════════════════════════════════════════════════ */
/*                       EKF_UpdateEngine                            */
/* ══════════════════════════════════════════════════════════════════ */
void EKF_UpdateEngine(EKF_t *ekf, float x_e, float y_e,
                      float vx_e, float vy_e, float theta_e)
{
    static float Pfull[EKF_N][EKF_N];
    static float H[5][EKF_N];
    static float PHt[EKF_N][5];    /* P · H^T (7×5) */
    static float S5[5][5];         /* H·P·H^T + R (5×5) */
    static float S5_inv[5][5];     /* inverse of S */
    static float K[EKF_N][5];      /* Kalman gain (7×5) */

    ekf_unpack_P(ekf->P, Pfull);

    float s = ekf->x[2];  /* sinθ */
    float c = ekf->x[3];  /* cosθ */
    float vx = ekf->x[4];
    float vy = ekf->x[5];
    float theta_cur = atan2f(s, c);

    /* ── Jacobian H (5×7) ── */
    memset(H, 0, sizeof(H));
    H[0][0] = 1.0f;                                      /* ∂h0/∂x */
    H[1][1] = 1.0f;                                      /* ∂h1/∂y */
    H[2][2] = -vy;  H[2][3] = vx;  H[2][4] = c;  H[2][5] = -s; /* ∂h2/∂(sinθ,cosθ,vx,vy) */
    H[3][2] = vx;   H[3][3] = vy;  H[3][4] = s;  H[3][5] = c;  /* ∂h3/∂(sinθ,cosθ,vx,vy) */
    H[4][2] = c;    H[4][3] = -s;                             /* ∂h4/∂(sinθ,cosθ) */

    /* ── Innovation ── */
    float y5[5];
    y5[0] = x_e - ekf->x[0];
    y5[1] = y_e - ekf->x[1];
    y5[2] = vx_e - (vx * c - vy * s);  /* vx_e is WORLD frame */
    y5[3] = vy_e - (vx * s + vy * c);  /* vy_e is WORLD frame */
    y5[4] = theta_e - theta_cur;

    /* Normalize angle innovation to [-π, π] */
    while (y5[4] > (float)M_PI)  y5[4] -= 2.0f * (float)M_PI;
    while (y5[4] < -(float)M_PI) y5[4] += 2.0f * (float)M_PI;

    /* ── PHt = P · H^T (7×5) ── */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_N; k++) {
                sum += Pfull[i][k] * H[j][k]; /* H^T[k][j] = H[j][k] */
            }
            PHt[i][j] = sum;
        }
    }

    /* ── S = H · P · H^T + R = H · PHt + R (5×5) ── */
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_N; k++) {
                sum += H[i][k] * PHt[k][j];
            }
            S5[i][j] = sum;
        }
        S5[i][i] += ekf->R_eng[i];
    }

    /* ── S inverse via Gauss-Jordan ── */
    /* Flatten S5 for the inverse routine */
    float S5_flat[25];
    float S5_inv_flat[25];
    for (uint8_t i = 0; i < 5; i++)
        for (uint8_t j = 0; j < 5; j++)
            S5_flat[i * 5 + j] = S5[i][j];

    if (inv_gauss_jordan(S5_flat, S5_inv_flat, 5) != 0) return; /* skip if singular */

    for (uint8_t i = 0; i < 5; i++)
        for (uint8_t j = 0; j < 5; j++)
            S5_inv[i][j] = S5_inv_flat[i * 5 + j];

    /* ── K = PHt · S^-1 (7×5) ── */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 5; k++) {
                sum += PHt[i][k] * S5_inv[k][j];
            }
            K[i][j] = sum;
        }
    }

    /* ── State update: x += K · y ── */
    for (uint8_t i = 0; i < EKF_N; i++) {
        float dx = 0.0f;
        for (uint8_t j = 0; j < 5; j++) {
            dx += K[i][j] * y5[j];
        }
        ekf->x[i] += dx;
    }

    /* ── Covariance: P = (I - K·H) · P ── */
    /* Compute KHP = K · (H · P), then P_new = P - KHP */
    {
        static float KHP[EKF_N][EKF_N];
        static float HP[5][EKF_N]; /* H · P (5×7) */

        /* HP = H · P (5×7) */
        for (uint8_t i = 0; i < 5; i++) {
            for (uint8_t j = 0; j < EKF_N; j++) {
                float sum = 0.0f;
                for (uint8_t k = 0; k < EKF_N; k++) {
                    sum += H[i][k] * Pfull[k][j];
                }
                HP[i][j] = sum;
            }
        }

        /* KHP = K · HP (7×5 × 5×7 → 7×7) */
        for (uint8_t i = 0; i < EKF_N; i++) {
            for (uint8_t j = 0; j < EKF_N; j++) {
                float sum = 0.0f;
                for (uint8_t k = 0; k < 5; k++) {
                    sum += K[i][k] * HP[k][j];
                }
                KHP[i][j] = sum;
            }
        }

        for (uint8_t i = 0; i < EKF_N; i++) {
            for (uint8_t j = 0; j < EKF_N; j++) {
                Pfull[i][j] -= KHP[i][j];
            }
        }
    }

    /* ── Enforce symmetry ── */
    for (uint8_t i = 0; i < EKF_N; i++) {
        for (uint8_t j = i + 1; j < EKF_N; j++) {
            float avg = 0.5f * (Pfull[i][j] + Pfull[j][i]);
            Pfull[i][j] = avg;
            Pfull[j][i] = avg;
        }
    }

    ekf_pack_P(ekf->P, Pfull);
}

/* ══════════════════════════════════════════════════════════════════ */
/*                       EKF_SaveHistory                             */
/* ══════════════════════════════════════════════════════════════════ */
void EKF_SaveHistory(EKF_t *ekf)
{
    uint8_t idx = ekf->hist_idx;

    memcpy(ekf->hist[idx], ekf->x, sizeof(float) * EKF_N);
    memcpy(ekf->hist_P[idx], ekf->P, sizeof(float) * EKF_NP);
    ekf->hist_time[idx] = ekf->time;

    ekf->hist_idx = (idx + 1) % EKF_HIST_SIZE;
    if (ekf->hist_count < EKF_HIST_SIZE)
        ekf->hist_count++;
}

/* ══════════════════════════════════════════════════════════════════ */
/*                   EKF_UpdateEngineDelayed                          */
/* ══════════════════════════════════════════════════════════════════ */
/* Applies a camera correction [x_cam, y_cam, theta_cam] that was valid
 * at time t_cam (in the EKF time base, seconds). Steps:
 *   1. Find the history entry closest to t_cam.
 *   2. Restore state x[] and covariance P[] from that entry.
 *   3. Apply the Kalman correction (EKF_UpdateEngine).
 *   4. Re-propagate the corrected state forward to the present using the
 *      current IMU inputs (ax_body, ay_body, gyro_z) — a good approximation
 *      over the short camera delay (15-30 ms).
 *
 * The caller must pass the latest IMU readings so re-propagation uses the
 * most recent acceleration/gyro available. */
void EKF_UpdateEngineDelayed(EKF_t *ekf, float x_cam, float y_cam,
                             float theta_cam, float t_cam,
                             float ax_body, float ay_body, float gyro_z)
{
    if (ekf->hist_count == 0) return; /* no history yet */

    /* ── 1. Find history index closest to t_cam ── */
    uint8_t best = 0;
    float best_dt = fabsf(ekf->hist_time[0] - t_cam);
    for (uint8_t i = 1; i < ekf->hist_count; i++) {
        float d = fabsf(ekf->hist_time[i] - t_cam);
        if (d < best_dt) {
            best_dt = d;
            best = i;
        }
    }

    /* ── 2. Restore state + covariance from history ── */
    memcpy(ekf->x, ekf->hist[best], sizeof(float) * EKF_N);
    memcpy(ekf->P, ekf->hist_P[best], sizeof(float) * EKF_NP);
    float t_hist = ekf->hist_time[best];

    /* ── 3. Apply camera correction at the historical time ── */
    /* vx_e/vy_e unknown from camera-only correction; pass current estimates
     * so the 5-state update does not disturb velocity from a position-only
     * measurement. We pass the restored velocities so innovation on vx/vy is 0. */
    EKF_UpdateEngine(ekf, x_cam, y_cam, ekf->x[4], ekf->x[5], theta_cam);

    /* ── 4. Re-propagate from t_hist to present ── */
    float remaining = ekf->time - t_hist;
    if (remaining > 0.0f && ekf->dt > 0.0f) {
        uint16_t steps = (uint16_t)(remaining / ekf->dt + 0.5f);
        if (steps > EKF_HIST_SIZE) steps = EKF_HIST_SIZE; /* safety cap */
        for (uint16_t i = 0; i < steps; i++) {
            EKF_Predict(ekf, ax_body, ay_body, gyro_z);
        }
    }
}
