// Compile with: g++ -std=c++11 MPCTest.cpp -o mpctest
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>

// --- Constants from MPC.cpp ---
const float PENDULUM_L = 0.35f;
const float GRAVITY = 9.81f;
const float DT = 0.005f;

// Your current tuning
const int N = 100; 
const int MAX_ITER = 200; 
const float LEARNING_RATE = 0.02f; 

const float maxAccel = 400000.0;
const float STEPS_PER_METER = 40000.0f;

struct MPCWeights {
    float q_pos = 60.0f;    
    float q_vel = 20.0f;   
    float q_theta = 500.0f; 
    float q_omega = 1.0f;
    float r_accel = 0.01f;
} mpc_cfg;

class MPCSolver {
public:
    std::vector<float> U; 

    MPCSolver() {
        U.resize(N, 0.0f);
    }

    float solve(float p_m, float v_m, float theta_rad, float omega_rad) {
        // Warm start shift (simulated)
        for (int i = 0; i < N - 1; ++i) U[i] = U[i+1];
        U[N-1] = 0;

        // std::cout << "Iter | Cost       | Grad Norm  | U[0]       | End Pos    | End Theta " << std::endl;
        // std::cout << "-----------------------------------------------------------------------" << std::endl;

        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // Forward pass
            struct StateVec { float p, v, t, w; };
            std::vector<StateVec> X(N + 1);
            X[0] = {p_m, v_m, theta_rad, omega_rad};

            float current_cost = 0.0f;

            for (int k = 0; k < N; ++k) {
                float u = U[k];
                X[k+1].p = X[k].p + X[k].v * DT;
                X[k+1].v = X[k].v + u * DT;
                X[k+1].t = X[k].t + X[k].w * DT;
                // Note: Using the POSITIVE sign convention established in previous chats
                X[k+1].w = X[k].w + (GRAVITY/PENDULUM_L * X[k].t + (1.0f/PENDULUM_L) * u) * DT;

                // Cost calculation
                current_cost += 0.5f * (mpc_cfg.q_pos * X[k+1].p * X[k+1].p + 
                                        mpc_cfg.q_vel * X[k+1].v * X[k+1].v + 
                                        mpc_cfg.q_theta * X[k+1].t * X[k+1].t + 
                                        mpc_cfg.q_omega * X[k+1].w * X[k+1].w + 
                                        mpc_cfg.r_accel * u * u);
            }

            // Backward pass
            StateVec lambda = {0, 0, 0, 0};
            float grad_norm = 0.0f;

            for (int k = N - 1; k >= 0; --k) {
                // Gradient w.r.t u_k
                float grad_u = mpc_cfg.r_accel * U[k] + (lambda.v * DT + lambda.w * (DT / PENDULUM_L));
                grad_norm += grad_u * grad_u;

                // Gradient Descent Step
                U[k] -= LEARNING_RATE * grad_u;

                // Clip
                float MAX_ACCEL_M = maxAccel / STEPS_PER_METER; 
                if (U[k] > MAX_ACCEL_M) U[k] = MAX_ACCEL_M;
                if (U[k] < -MAX_ACCEL_M) U[k] = -MAX_ACCEL_M;

                // Update Lambda
                StateVec Qx;
                Qx.p = mpc_cfg.q_pos * X[k+1].p;
                Qx.v = mpc_cfg.q_vel * X[k+1].v;
                Qx.t = mpc_cfg.q_theta * X[k+1].t; 
                Qx.w = mpc_cfg.q_omega * X[k+1].w;

                StateVec next_lambda;
                next_lambda.p = Qx.p + lambda.p; 
                next_lambda.v = Qx.v + lambda.p * DT + lambda.v;
                next_lambda.t = Qx.t + lambda.t + lambda.w * (GRAVITY/PENDULUM_L * DT);
                next_lambda.w = Qx.w + lambda.t * DT + lambda.w;
                
                lambda = next_lambda; 
            }

            // if (iter % 5 == 0 || iter == MAX_ITER - 1) {
            //     std::cout << std::setw(4) << iter << " | " 
            //               << std::setw(10) << current_cost << " | " 
            //               << std::setw(10) << std::sqrt(grad_norm) << " | " 
            //               << std::setw(10) << U[0] << " | "
            //               << std::setw(10) << X[N].p << " | "
            //               << std::setw(10) << X[N].t << std::endl;
            // }
        }

        // // Print Final Trajectory
        // std::cout << "\nFinal Predicted Trajectory:" << std::endl;
        // std::cout << "Step | Pos (m) | Vel (m/s) | Theta (rad) | Omega (rad/s) | Accel (m/s^2)" << std::endl;
        
        // struct StateVec { float p, v, t, w; };
        // StateVec x = {p_m, v_m, theta_rad, omega_rad};
        
        // for (int k = 0; k < N; ++k) {
        //     float u = U[k];
        //     std::cout << std::setw(4) << k << " | "
        //               << std::setw(7) << std::fixed << std::setprecision(4) << x.p << " | "
        //               << std::setw(9) << x.v << " | "
        //               << std::setw(11) << x.t << " | "
        //               << std::setw(13) << x.w << " | "
        //               << std::setw(12) << u << std::endl;

        //     float next_p = x.p + x.v * DT;
        //     float next_v = x.v + u * DT;
        //     float next_t = x.t + x.w * DT;
        //     float next_w = x.w + (GRAVITY/PENDULUM_L * x.t + (1.0f/PENDULUM_L) * u) * DT;
        //     x = {next_p, next_v, next_t, next_w};
        // }

        return U[0];
    }
};

int main() {
    MPCSolver mpc;
    // Test Case: Pendulum leaning Right (-5 degrees), Cart at Center
    float initial_theta = -0.0f * (M_PI / 180.0f); 
    float initial_omega = 0.0f;
    float initial_pos = 0.2f;
    float initial_vel = 0.0f;   
    std::cout << "Test Case: Theta = " << initial_theta << " rad (-5 deg)" << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    mpc.solve(initial_pos, initial_vel, initial_theta, initial_omega);
    auto t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Time to solve (s): " << time_span.count() << std::endl;

    return 0;
}