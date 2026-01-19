// Compile with: g++ -pthread -std=c++11 PID.cpp
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <signal.h>

// --- FPGA Memory Map ---
#define LWHPS2FPGA_BASE 0xFF200000
#define LWHPS2FPGA_SPAN 0x00000040 // 64 bytes (16 registers)

// --- HPS Reset Manager (for Bridge Enable) ---
#define RSTMGR_BASE      0xFFD05000
#define RSTMGR_SPAN      0x1000
#define RSTMGR_BRGMODRST 0x1C

// Register Offsets (calculated from your 4-byte/32-bit registers)
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
const float maxSwingUpAccel = 100000.0;
const float maxMotorSpeed = 100000.0;
const float FALL_LIMIT = 30.0;
const float MAX_TARGET_ANGLE = 12.0;

const int total_track_steps = 26120;
const int safety_margin = 300;
const int min_limit = -total_track_steps / 2 + safety_margin;
const int max_limit = total_track_steps / 2 - safety_margin;


// PID Gains for maxAccel 100000 
// float angle_Kp = 14000.0, angle_Kd = 2000.0, angle_Ki = 0.0;
// float pos_Kp = 0.00020, pos_Kd = 0.0002, pos_Ki = 0.0;

// PID Gains for maxAccel 400000
float angle_Kp = 10000.0, angle_Kd = 2000.0, angle_Ki = 0.0;
float pos_Kp = 0.00020, pos_Kd = 0.0004, pos_Ki = 0.0;

std::mutex gain_mutex;

// Constants for Energy Control
const float K_ENERGY = 1.8f;      // Gain for energy pumping (tune this)
const float PENDULUM_L = 0.35f;   // Length in meters (pivot to center of mass)
const float GRAVITY = 9.81f;      
const float UP_THRESHOLD = 20.0f; // Angle in degrees from vertical to catch

// Global state
void* virtual_base;
int i2c_fd;

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
        // Clear bit 1 (LWHPS2FPGA) to release bridge from reset
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
    messages[0].flags = 0; // Write
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = AS5048_ADDR;
    messages[1].flags = I2C_M_RD; // Read
    messages[1].len = 2;
    messages[1].buf = data;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &packets) < 0) return 0;

    return ((uint16_t)data[0] << 6) | (data[1] & 0x3F);
}

float getPhysicalAngle(float initialAngle, float& filteredAngle) {
    // const float alpha = 0.7;
    uint16_t raw = read14BitAngle();
    float degrees = raw * 0.02197265625;
    float currentRaw = degrees - initialAngle - 180.0;

    if (currentRaw > 180.0) currentRaw -= 360.0;
    if (currentRaw < -180.0) currentRaw += 360.0;

    // filteredAngle = (alpha * currentRaw) + ((1.0 - alpha) * filteredAngle);
    // return filteredAngle;
    return currentRaw;
}

void inputLoop() {
    std::string line, cmd;
    float val;
    std::cout << "PID Tuner: Enter <param> <value> (e.g., akp 15000)" << std::endl;
    while (std::getline(std::cin, line)) {
        std::stringstream ss(line);
        if (ss >> cmd >> val) {
            std::lock_guard<std::mutex> lock(gain_mutex);
            if (cmd == "akp") angle_Kp = val;
            else if (cmd == "akd") angle_Kd = val;
            else if (cmd == "aki") angle_Ki = val;
            else if (cmd == "pkp") pos_Kp = val;
            else if (cmd == "pkd") pos_Kd = val;
            else if (cmd == "pki") pos_Ki = val;
            else std::cout << "Unknown param: " << cmd << std::endl;
        }
    }
}

int main() {
    // Register signal handlers for Ctrl-C and crashes
    signal(SIGINT, signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);

    std::cout << "Starting..." << std::endl; 

    // 0. Enable HPS-to-FPGA Bridge
    enable_lwhps_bridge();

    std::cout << "Initializing Memory Mapping..." << std::endl;
    // 1. Initialize Memory Mapping
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    virtual_base = mmap(NULL, LWHPS2FPGA_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, LWHPS2FPGA_BASE);

    std::cout << "Initializing I2C..." << std::endl;
    // 2. Initialize I2C
    i2c_fd = open("/dev/i2c-1", O_RDWR); // Change to i2c-0 if needed
    ioctl(i2c_fd, I2C_SLAVE, AS5048_ADDR);

    
    std::cout << "Setting up registers..." << std::endl;
    // 3. Setup FPGA Stepper Params
    const int homingAccel = 100000;
    const int homingMaxSpeed = 10000;
    write_reg(REG_ACCEL_VAL, (int32_t)homingAccel);
    write_reg(REG_MAX_SPEED, (int32_t)homingMaxSpeed);

    // 4. Homing (Simplified logic)
    std::cout << "Homing..." << std::endl;

    // Clear limits with min and max value of int
    write_reg(REG_MIN_LIMIT, 0x80000000);
    write_reg(REG_MAX_LIMIT, 0x7FFFFFFF);
    write_reg(REG_TARGET_SPEED, (int32_t)homingSpeed);
    while (read_reg(REG_HOME_SWITCH) == 0) usleep(1000);
    write_reg(REG_TARGET_SPEED, 0);

    std::cout << "Setting limits..." << std::endl;
    // Set 0 position
    write_reg(REG_POS_COUNTER, 0);
    write_reg(REG_TARGET_POS, (int32_t)(total_track_steps / 2));

    // Wait until it reaches middle
    while(read_reg(REG_POS_COUNTER) < (total_track_steps / 2)) usleep(1000);
    write_reg(REG_POS_COUNTER, 0);
    write_reg(REG_TARGET_POS, 0);
    write_reg(REG_MIN_LIMIT, (int32_t)min_limit);
    write_reg(REG_MAX_LIMIT, (int32_t)max_limit);
    
    // Stabilize and get initial angle
    // sleep(4);
    // float initialAngle = read14BitAngle() * 0.02197265625;
    float initialAngle = 331.26;
    float filteredAngle = 0, angle_integral = 0, angle_lastError = 0;
    float pos_integral = 0, pos_lastError = 0, pid_pos_out = 0;

    std::cout << "Initial Angle: " << initialAngle << std::endl;
    std::cout << "Ready to stabilize..." << std::endl;

    auto lastTime = std::chrono::high_resolution_clock::now();


    write_reg(REG_ACCEL_VAL, (int32_t)maxAccel);
    write_reg(REG_MAX_SPEED, (int32_t)maxMotorSpeed);

    // Start input thread
    std::thread t(inputLoop);
    t.detach();

    enum ControlState { IDLE, SWING_UP, STABILIZE };
    ControlState currentState = SWING_UP; // Start in swing-up
    write_reg(REG_ACCEL_VAL, (int32_t)maxSwingUpAccel);


    // Velocity tracking
    float lastAngle = 0;
    float angularVelocity = 0;
    bool fell = false;

    // 5. Main Control Loop
    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = now - lastTime;
        float dt = elapsed.count();

        if (dt >= 0.005) { // 5ms (200Hz)
            lastTime = now;

            float cur_angle_Kp, cur_angle_Kd, cur_angle_Ki;
            float cur_pos_Kp, cur_pos_Kd, cur_pos_Ki;
            {
                std::lock_guard<std::mutex> lock(gain_mutex);
                cur_angle_Kp = angle_Kp; cur_angle_Kd = angle_Kd; cur_angle_Ki = angle_Ki;
                cur_pos_Kp = pos_Kp; cur_pos_Kd = pos_Kd; cur_pos_Ki = pos_Ki;
            }

            // Get State Variables
            float currentAngle = getPhysicalAngle(initialAngle, filteredAngle);
            int32_t currentPos = read_reg(REG_POS_COUNTER);

            // --- Global Velocity Calculation ---
            float deltaAngle = currentAngle - lastAngle;
            if (deltaAngle > 180.0f)  deltaAngle -= 360.0f;
            if (deltaAngle < -180.0f) deltaAngle += 360.0f;
            float velocity_deg_s = deltaAngle / dt;
            lastAngle = currentAngle;

            // Filtered Omega for D-Term (Low-pass filter to reduce noise)
            static float filtered_vel_deg_s = 0;
            // filtered_vel_deg_s = 0.7f * filtered_vel_deg_s + 0.3f * velocity_deg_s;
            filtered_vel_deg_s = velocity_deg_s;

            float motorSpeed = 0;

            // 2. State Machine Logic
            switch (currentState) {
                
                case SWING_UP: {
                    // 1. Velocity (converted to radians for energy math)
                    float raw_omega = velocity_deg_s * (M_PI / 180.0f);

                    // Filtered velocity for energy math
                    static float filtered_omega = 0;
                    if (std::abs(raw_omega) < 50.0f) filtered_omega = (0.8f * filtered_omega) + (0.2f * raw_omega);

                    // 2. Energy Calculation
                    float theta = currentAngle * (M_PI / 180.0f);
                    float mgh_l = (GRAVITY / PENDULUM_L);
                    float current_pe = mgh_l * cos(theta);
                    float current_ke = 0.5f * filtered_omega * filtered_omega;
                    

                    // Add an offset to compensate for friction losses (air resistance, bearings)
                    float energy_offset = 15.0f; 
                    float energy_error = (mgh_l + energy_offset) - (current_ke + current_pe);

                    // 3. The "Energy Pump" Control Law
                    // K_SWING determines how aggressively the cart moves. 
                    // Variable gain: Start high (~500) when error is large, decrease to 50 near target.
                    // Small K after a fall
                    float K_SWING = (fell) ? 100.0f : 200.0f;

                    
                    if (energy_error > 0) {
                        // Only pump energy if we haven't reached the target yet
                        motorSpeed = K_SWING * energy_error * (filtered_omega * cos(theta));
                    } else {
                        // We have enough energy! 
                        // If we are just slightly over (within 5J), coast to let momentum carry it.
                        // Only brake if we have significantly too much energy.
                        fell = false;
                        if (energy_error > -5.0f) {
                            motorSpeed = 0.0f;
                        } else {
                            motorSpeed = -10.0f * filtered_omega;
                        }
                    }

                    // Add centering force to keep swing-up in the middle of the track
                    float K_CENTER = 0.8f; 
                    motorSpeed -= K_CENTER * currentPos;

                    // 2. The Catch (Transition to PID)
                    // Criteria: Pendulum is within 20 degrees of vertical AND moving slow enough to catch
                    if (std::abs(currentAngle) < 20.0f && std::abs(filtered_omega) < 10.0f) {
                        // CRITICAL: Reset PID integrals before switching to avoid a "jump"
                        angle_integral = 0;
                        pos_integral = 0;
                        angle_lastError = 0;
                        pos_lastError = 0;
                        
                        currentState = STABILIZE;
                        write_reg(REG_ACCEL_VAL, (int32_t)maxAccel);
                        printf("\n--- PENDULUM CAUGHT! Switching to STABILIZE ---\n");
                    }

                    // 4. Safety Limits
                    if (motorSpeed > maxMotorSpeed) motorSpeed = maxMotorSpeed;
                    if (motorSpeed < -maxMotorSpeed) motorSpeed = -maxMotorSpeed;

                    // Boundary check: If we are hitting the edges of the track, kill the speed
                    int32_t currentPos = read_reg(REG_POS_COUNTER);
                    if ((currentPos > (max_limit - 2000) && motorSpeed > 0) || 
                        (currentPos < (min_limit + 2000) && motorSpeed < 0)) {
                        motorSpeed = 0;
                    }

                    // --- TELEMETRY ---
                    static int print_divider = 0;
                    if (print_divider++ % 20 == 0) {
                        printf("Err: %6.2f | Speed: %6.0f | Pos: %d\n", energy_error, motorSpeed, currentPos);
                    }
                    
                    write_reg(REG_TARGET_SPEED, (int32_t)motorSpeed);
                    break;
                }

                case STABILIZE: {
                    // Your existing PID logic
                    float cur_angle_Kp, cur_angle_Kd, cur_angle_Ki;
                    float cur_pos_Kp, cur_pos_Kd, cur_pos_Ki;
                    {
                        std::lock_guard<std::mutex> lock(gain_mutex);
                        cur_angle_Kp = angle_Kp; cur_angle_Kd = angle_Kd; cur_angle_Ki = angle_Ki;
                        cur_pos_Kp = pos_Kp; cur_pos_Kd = pos_Kd; cur_pos_Ki = pos_Ki;
                    }

                    // Outer Loop (Position)
                    float pos_error = (float)currentPos; 
                    pos_integral += pos_error * dt;
                    float pos_derivative = (pos_error - pos_lastError) / dt;
                    pos_lastError = pos_error;
                    pid_pos_out = (cur_pos_Kp * pos_error) + (cur_pos_Ki * pos_integral) + (cur_pos_Kd * pos_derivative);
                    float targetAngle = std::max(-MAX_TARGET_ANGLE, std::min(MAX_TARGET_ANGLE, pid_pos_out));

                    // Inner Loop (Angle)
                    float angle_error = targetAngle - currentAngle;
                    angle_integral += angle_error * dt;

                    // Derivative on Measurement: Use -Kd * Omega
                    // This avoids "Derivative Kick" when the targetAngle changes rapidly
                    motorSpeed = (cur_angle_Kp * angle_error) + (cur_angle_Ki * angle_integral) - (cur_angle_Kd * filtered_vel_deg_s);

                    // Transition: If it falls beyond the limit, go back to swing up
                    if (std::abs(currentAngle) > FALL_LIMIT) {
                        currentState = SWING_UP;
                        write_reg(REG_ACCEL_VAL, (int32_t)maxSwingUpAccel); 
                        fell = true;
                        std::cout << "Fell! Restarting SWING_UP" << std::endl;
                    }
                    break;
                }
            }

            // 3. Hardware Safety & Output
            // Prevent rail-smash during swingup
            if (currentPos > (max_limit - 1000) && motorSpeed > 0) motorSpeed = -2000;
            if (currentPos < (min_limit + 1000) && motorSpeed < 0) motorSpeed = 2000;

            if (motorSpeed > maxMotorSpeed) motorSpeed = maxMotorSpeed;
            if (motorSpeed < -maxMotorSpeed) motorSpeed = -maxMotorSpeed;

            write_reg(REG_TARGET_SPEED, (int32_t)motorSpeed);
        }
    }


    // Cleanup
    munmap(virtual_base, LWHPS2FPGA_SPAN);
    close(mem_fd);
    close(i2c_fd);
    return 0;
}