// Compile with: g++ -pthread -std=c++11 -O3 MPC.cpp -o mpc
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <signal.h>

// --- FPGA Memory Map ---
#define LWHPS2FPGA_BASE 0xFF200000
#define LWHPS2FPGA_SPAN 0x00000040 

// --- HPS Reset Manager ---
#define RSTMGR_BASE      0xFFD05000
#define RSTMGR_SPAN      0x1000
#define RSTMGR_BRGMODRST 0x1C

// Register Offsets
#define REG_TARGET_SPEED 0x00
#define REG_TARGET_POS   0x04
#define REG_ACCEL_VAL    0x08
#define REG_MAX_SPEED    0x0C
#define REG_MIN_LIMIT    0x10
#define REG_MAX_LIMIT    0x14
#define REG_CURR_SPEED   0x18
#define REG_POS_COUNTER  0x1C
#define REG_HOME_SWITCH  0x20

// --- Hardware Settings ---
const int AS5048_ADDR = 0x40;
const float homingSpeed = -3000.0;
const float maxAccel = 400000.0;
const float maxSwingUpAccel = 75000.0;
const float maxMotorSpeed = 100000.0;
const float FALL_LIMIT = 30.0;

const int total_track_steps = 26120;
const int safety_margin = 300;
const int min_limit = -total_track_steps / 2 + safety_margin;
const int max_limit = total_track_steps / 2 - safety_margin;

// --- Physical Constants ---
const float PENDULUM_L = 0.35f;
const float GRAVITY = 9.81f;
const float DT = 0.005f; // 5ms control loop

// --- MPC Configuration ---
const int N = 100; 
const int MAX_ITER = 200; 
const float LEARNING_RATE = 0.01f; 

struct MPCWeights {
    float q_pos = 200.0f;    
    float q_vel = 1.0f;   
    float q_theta = 50.0f; 
    float q_omega = 0.1f;
    float r_accel = 0.01f;
} mpc_cfg;

std::mutex gain_mutex;

// Global state
void* virtual_base;
int i2c_fd;

// --- Helper Math ---
struct State {
    float p; // Position (steps)
    float v; // Velocity (steps/s)
    float t; // Theta (rad)
    float w; // Omega (rad/s)
};

// --- FPGA Register Helpers ---
void write_reg(uint32_t offset, int32_t value) {
    *(volatile int32_t*)((uint8_t*)virtual_base + offset) = value;
}

int32_t read_reg(uint32_t offset) {
    return *(volatile int32_t*)((uint8_t*)virtual_base + offset);
}

void signalHandler(int signum) {
    if (virtual_base != MAP_FAILED && virtual_base != nullptr) {
        write_reg(REG_TARGET_SPEED, 0);
    }
    std::cout << "\nCaught signal " << signum << ", stopping motor..." << std::endl;
    exit(signum);
}

// --- Bridge Control ---
void enable_lwhps_bridge() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return;
    void* rstmgr = mmap(NULL, RSTMGR_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, RSTMGR_BASE);
    if (rstmgr != MAP_FAILED) {
        uint32_t* brgmodrst = (uint32_t*)((uint8_t*)rstmgr + RSTMGR_BRGMODRST);
        *brgmodrst &= ~(0x2);
        munmap(rstmgr, RSTMGR_SPAN);
    }
    close(fd);
}

// --- I2C Sensor Reading ---
uint16_t read14BitAngle() {
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    uint8_t reg = 0xFE;
    uint8_t data[2] = {0};

    messages[0].addr = AS5048_ADDR;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = AS5048_ADDR;
    messages[1].flags = I2C_M_RD;
    messages[1].len = 2;
    messages[1].buf = data;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &packets) < 0) return 0;
    return ((uint16_t)data[0] << 6) | (data[1] & 0x3F);
}

float getPhysicalAngle(float initialAngle) {
    uint16_t raw = read14BitAngle();
    float degrees = raw * 0.02197265625;
    float currentRaw = degrees - initialAngle - 180.0;
    if (currentRaw > 180.0) currentRaw -= 360.0;
    if (currentRaw < -180.0) currentRaw += 360.0;
    return currentRaw;
}

// --- MPC Solver Class ---
// Solves the QP: min 0.5 * U^T * H * U + G^T * U
// Subject to input constraints (acceleration limits)
class MPCSolver {
public:
    // Flattened matrices for the QP problem
    // H is N x N, G is N x 4 (multiplied by state x0)
    std::vector<float> H; 
    std::vector<float> K; // K = 2 * B_bar^T * Q_bar * A_bar
    std::vector<float> U; // Current solution (warm start)
    
    // Model matrices (Discrete)
    // x_{k+1} = A x_k + B u_k
    // A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 g/L*dt 1]
    // B = [0; dt; 0; 1/L*dt] (Input u is acceleration)
    
    // Steps to meters conversion (approximate, for model consistency)
    // Assuming 1 meter approx 10000 steps for the physics model, 
    // but we can just run the model in "steps" and "degrees" if we scale Q/R appropriately.
    // Let's stick to SI units for the internal model and convert I/O.
    // TODO: Tune this for your hardware! 
    // Calculation: (Steps/Rev * Microstepping) / (Pulley_Teeth * Pitch_mm / 1000)
    // Example: (200 * 16) / (20 * 2 / 1000) = 3200 / 0.04 = 80,000 steps/m

    const float STEPS_PER_METER = 40000.0f; 
    const float DEG_TO_RAD = M_PI / 180.0f;

public:

    MPCSolver() {
        U.resize(N, 0.0f);
    }

    // Solves for the next control input u (acceleration in m/s^2)
    float solve(float p_m, float v_m, float theta_rad, float omega_rad) {
        std::lock_guard<std::mutex> lock(gain_mutex);
        
        // Shift previous solution for warm start
        for (int i = 0; i < N - 1; ++i) U[i] = U[i+1];
        U[N-1] = 0;

        // Gradient Descent
        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // Forward simulation to compute cost and gradients
            // We use the Adjoint Method (Backpropagation) for O(N) complexity
            
            // Forward pass: Store states
            struct StateVec { float p, v, t, w; };
            std::vector<StateVec> X(N + 1);
            X[0] = {p_m, v_m, theta_rad, omega_rad};

            for (int k = 0; k < N; ++k) {
                float u = U[k];
                X[k+1].p = X[k].p + X[k].v * DT;
                X[k+1].v = X[k].v + u * DT;
                X[k+1].t = X[k].t + X[k].w * DT;
                X[k+1].w = X[k].w + (GRAVITY/PENDULUM_L * X[k].t + (1.0f/PENDULUM_L) * u) * DT;
            }

            // Backward pass: Compute co-states (lambda) and gradient
            // Cost J = sum( 0.5*x'Qx + 0.5*u'Ru )
            // Co-state eq: lambda_k = Q*x_k + A'*lambda_{k+1}
            // Grad_u = R*u + B'*lambda_{k+1}
            
            StateVec lambda = {0, 0, 0, 0}; // lambda_N (terminal cost = 0 for now)

            for (int k = N - 1; k >= 0; --k) {
                // 1. Compute Gradient w.r.t u_k
                // B' * lambda = lambda.v * dt + lambda.w * (dt/L)
                float grad_u = mpc_cfg.r_accel * U[k] + (lambda.v * DT + lambda.w * (DT / PENDULUM_L));

                // 2. Update U_k (Gradient Descent Step)
                U[k] -= LEARNING_RATE * grad_u;

                // 3. Clip U_k (Hard Constraints on Acceleration)
                // Max accel 100000 steps/s^2 approx 20 m/s^2
                // Fix: Match physical limit (100000 / 40000 = 2.5 m/s^2)
                float MAX_ACCEL_M = maxAccel / STEPS_PER_METER; 
                if (U[k] > MAX_ACCEL_M) U[k] = MAX_ACCEL_M;
                if (U[k] < -MAX_ACCEL_M) U[k] = -MAX_ACCEL_M;

                // 4. Update Lambda for next step (k-1)
                // lambda_{k} = Q*x_{k+1} + A'*lambda_{k+1}
                // Note: We use x_{k+1} because cost is on next state
                StateVec Qx;
                Qx.p = mpc_cfg.q_pos * X[k+1].p;
                Qx.v = mpc_cfg.q_vel * X[k+1].v;
                Qx.t = mpc_cfg.q_theta * X[k+1].t;
                Qx.w = mpc_cfg.q_omega * X[k+1].w;

                StateVec next_lambda;
                // A transpose multiply
                // p_prev depends on p (1)
                // v_prev depends on p (dt), v (1)
                // t_prev depends on t (1), w (g/L*dt)
                // w_prev depends on t (dt), w (1)
                next_lambda.p = Qx.p + lambda.p; 
                next_lambda.v = Qx.v + lambda.p * DT + lambda.v;
                next_lambda.t = Qx.t + lambda.t + lambda.w * (GRAVITY/PENDULUM_L * DT);
                next_lambda.w = Qx.w + lambda.t * DT + lambda.w;
                
                lambda = next_lambda;
            }
        }

        return U[0];
    }
};

void inputLoop() {
    std::string line, cmd;
    float val;
    std::cout << "MPC Tuner: <param> <value> (e.g., qtheta 200)" << std::endl;
    std::cout << "Params: qpos, qvel, qtheta, qomega, raccel" << std::endl;
    while (std::getline(std::cin, line)) {
        std::stringstream ss(line);
        if (ss >> cmd >> val) {
            std::lock_guard<std::mutex> lock(gain_mutex);
            if (cmd == "qpos") mpc_cfg.q_pos = val;
            else if (cmd == "qvel") mpc_cfg.q_vel = val;
            else if (cmd == "qtheta") mpc_cfg.q_theta = val;
            else if (cmd == "qomega") mpc_cfg.q_omega = val;
            else if (cmd == "raccel") mpc_cfg.r_accel = val;
            else std::cout << "Unknown param: " << cmd << std::endl;
        }
    }
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);

    std::cout << "Starting MPC Controller..." << std::endl; 
    enable_lwhps_bridge();

    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    virtual_base = mmap(NULL, LWHPS2FPGA_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, LWHPS2FPGA_BASE);

    i2c_fd = open("/dev/i2c-1", O_RDWR);
    ioctl(i2c_fd, I2C_SLAVE, AS5048_ADDR);

    // --- Homing ---
    std::cout << "Homing..." << std::endl;
    write_reg(REG_ACCEL_VAL, 100000);
    write_reg(REG_MAX_SPEED, 10000);
    write_reg(REG_MIN_LIMIT, 0x80000000);
    write_reg(REG_MAX_LIMIT, 0x7FFFFFFF);
    write_reg(REG_TARGET_SPEED, (int32_t)homingSpeed);
    while (read_reg(REG_HOME_SWITCH) == 0) usleep(1000);
    write_reg(REG_TARGET_SPEED, 0);

    write_reg(REG_POS_COUNTER, 0);
    write_reg(REG_TARGET_POS, (int32_t)(total_track_steps / 2));
    while(read_reg(REG_POS_COUNTER) < (total_track_steps / 2)) usleep(1000);
    write_reg(REG_POS_COUNTER, 0);
    write_reg(REG_TARGET_POS, 0);
    write_reg(REG_MIN_LIMIT, (int32_t)min_limit);
    write_reg(REG_MAX_LIMIT, (int32_t)max_limit);
    
    // float initialAngle = read14BitAngle() * 0.02197265625;
    float initialAngle = 331.65;  //331.26;// Calibrated UP value
    std::cout << "Initial Angle: " << initialAngle << std::endl;

    write_reg(REG_ACCEL_VAL, (int32_t)maxAccel);
    write_reg(REG_MAX_SPEED, (int32_t)maxMotorSpeed);

    std::thread t(inputLoop);
    t.detach();

    MPCSolver mpc;
    
    enum ControlState { SWING_UP, STABILIZE };
    ControlState currentState = SWING_UP;
    write_reg(REG_ACCEL_VAL, (int32_t)maxSwingUpAccel); // Smaller max accel for swing up
    // ControlState currentState = STABILIZE;

    float lastAngle = 0;
    float currentSpeedCmd = 0; // Current velocity command sent to motor
    bool fell = false;

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = now - lastTime;
        float dt = elapsed.count();

        if (dt >= DT) {
            lastTime = now;

            float currentAngle = getPhysicalAngle(initialAngle);
            int32_t currentPosSteps = read_reg(REG_POS_COUNTER);

            // Calculate Angular Velocity
            float deltaAngle = currentAngle - lastAngle;
            if (deltaAngle > 180.0f)  deltaAngle -= 360.0f;
            if (deltaAngle < -180.0f) deltaAngle += 360.0f;
            float raw_omega = (deltaAngle / dt) * (M_PI / 180.0f);
            lastAngle = currentAngle;

            // Filter Omega
            static float filtered_omega = 0;
            filtered_omega = (0.7f * filtered_omega) + (0.3f * raw_omega);

            float motorSpeed = 0;

            if (currentState == SWING_UP) {
                // --- ENERGY SHAPING (Same as PID version) ---
                float theta = currentAngle * (M_PI / 180.0f);
                float mgh_l = (GRAVITY / PENDULUM_L);
                float current_pe = mgh_l * cos(theta);
                float current_ke = 0.5f * filtered_omega * filtered_omega;
                float energy_offset = 15.0f; 
                float energy_error = (mgh_l + energy_offset) - (current_ke + current_pe);
                float K_SWING = (fell) ? 100.0f : 200.0f;

                if (energy_error > 0) {
                    motorSpeed = K_SWING * energy_error * (filtered_omega * cos(theta));
                } else {
                    if (energy_error > -5.0f) motorSpeed = 0.0f;
                    else motorSpeed = -10.0f * filtered_omega;
                }

                // Add centering force to keep swing-up in the middle of the track
                float K_CENTER = 0.8f; 
                motorSpeed -= K_CENTER * currentPosSteps;

                // Transition to MPC
                if (std::abs(currentAngle) < 20.0f && std::abs(filtered_omega) < 8.0f) {
                    currentState = STABILIZE;
                    write_reg(REG_ACCEL_VAL, (int32_t)maxAccel);
                    currentSpeedCmd = motorSpeed; // Handover velocity
                    printf("\n--- CAUGHT! Switching to MPC ---\n");
                }

            } else {
                // --- MPC STABILIZATION ---
                
                // Convert units for MPC (Steps -> Meters, Degrees -> Radians)
                // 5000 steps/m is an approximation, tune STEPS_PER_METER if needed
                // Invert position to fix positive feedback loop (runaway cart)
                float p_m = currentPosSteps / mpc.STEPS_PER_METER; 
                float v_m = currentSpeedCmd / mpc.STEPS_PER_METER;
                float theta_rad = currentAngle * (M_PI / 180.0f);
                float omega_rad = filtered_omega;

                // Solve for optimal acceleration (m/s^2)
                // auto before_solve = std::chrono::high_resolution_clock::now();
                float accel_cmd_m = mpc.solve(p_m, v_m, theta_rad, omega_rad);
                // auto after_solve = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<float> solve_time = after_solve - before_solve;
                // std::cout << "Solve Time: " << solve_time.count() << "s" << std::endl;


                // Integrate acceleration to get velocity command
                // v_new = v_old + a * dt
                float v_new_m = v_m + accel_cmd_m * DT;
                
                // Convert back to steps/s
                motorSpeed = v_new_m * mpc.STEPS_PER_METER;

                // static int debug_cnt = 0;
                // if (debug_cnt++ % 20 == 0) {
                //     printf("MPC: Ang=%.2f Om=%.2f Pos=%.2f Vel=%.2f Acc=%.2f Spd=%.0f\n", 
                //         currentAngle, filtered_omega, p_m, v_m, accel_cmd_m, motorSpeed);
                // }

                // Fall detection
                if (std::abs(currentAngle) > FALL_LIMIT) {
                    currentState = SWING_UP;
                    write_reg(REG_ACCEL_VAL, (int32_t)maxSwingUpAccel);
                    fell = true;
                    std::cout << "Fell! Restarting SWING_UP" << std::endl;
                    // motorSpeed = 0;
                }
            }

            // --- Safety & Output ---
            if (motorSpeed > maxMotorSpeed) motorSpeed = maxMotorSpeed;
            if (motorSpeed < -maxMotorSpeed) motorSpeed = -maxMotorSpeed;

            // Rail safety
            if (currentPosSteps > (max_limit - 1000) && motorSpeed > 0) motorSpeed = -2000;
            if (currentPosSteps < (min_limit + 1000) && motorSpeed < 0) motorSpeed = 2000;

            // Update internal state with the actual clamped value
            currentSpeedCmd = motorSpeed;

            write_reg(REG_TARGET_SPEED, (int32_t)motorSpeed);
        }
    }

    munmap(virtual_base, LWHPS2FPGA_SPAN);
    close(mem_fd);
    close(i2c_fd);
    return 0;
}
