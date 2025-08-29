#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

// Constants
const int N = 1000;
const double L = 10.0;
const double R = 1.0;
const double v0 = 0.3;
const double eta = 0.5;
const double dt = 1.0;
const int steps = 1000;

// Random number generator
std::mt19937 rng(42);
std::uniform_real_distribution<double> uniform_angle(0.0, 2.0 * M_PI);
std::uniform_real_distribution<double> uniform_noise(-eta / 2.0, eta / 2.0);

// Agent with direction as a vector (vx, vy, vz)
struct Agent{
    double x, y, z; // positions in 3D
    double vx, vy, vz; // velocities in 3D 

    void move();
    void normalize();
    void apply_noise();
};


void Agent::move(){
    x += v0 * vx;
    y += v0 * vy;
    z += v0 * vz;


    // 3D periodic BC 
    if (x < 0) x += L; if (x >= L) x -= L;
    if (y < 0) y += L; if (y >= L) y -= L;
    if (z < 0) z += L; if (z >= L) z -= L;
}

void Agent::normalize() {
        double mag = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (mag > 0) {
            vx /= mag;
            vy /= mag;
            vz /= mag;
        }
}

void Agent::apply_noise(){
        double phi = uniform_angle(rng);             // 0 to 2pi
        double cos_r = std::uniform_real_distribution<double>(-1, 1)(rng);
        double sin_r = std::sqrt(1 - cos_r * cos_r);
        double rx = sin_r * std::cos(phi);
        double ry = sin_r * std::sin(phi);
        double rz = cos_r;

    // Project r perpendicular to v_avg
        double dot = rx * vx + ry * vy + rz * vz;
        double kx = rx - dot * vx;
        double ky = ry - dot * vy;
        double kz = rz - dot * vz;

    // Normalize k
    double mag_k = std::sqrt(kx * kx + ky * ky + kz * kz);
    if (mag_k < 1e-8) return;  // skip if degenerate
    kx /= mag_k;
    ky /= mag_k;
    kz /= mag_k;

    // Step 3: Sample small random rotation angle theta (noise)
    double theta = uniform_noise(rng);  // e.g., from [-η/2, η/2]

    // Step 4: Rodrigues' rotation
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    double v_dot_k = kx * vx + ky * vy + kz * vz;

    double new_vx = vx * cos_theta +
                    (ky * vz - kz * vy) * sin_theta +
                    kx * v_dot_k * (1.0 - cos_theta);

    double new_vy = vy * cos_theta +
                    (kz * vx - kx * vz) * sin_theta +
                    ky * v_dot_k * (1.0 - cos_theta);

    double new_vz = vz * cos_theta +
                    (kx * vy - ky * vx) * sin_theta +
                    kz * v_dot_k * (1.0 - cos_theta);

    // Step 5: Update and normalize
    vx = new_vx;
    vy = new_vy;
    vz = new_vz;
    normalize();
}


// Compute distance with periodic boundaries
double distance(double x1, double y1, double z1, double x2, double y2, double z2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dz = z1 - z2;


    if (dx > L / 2) dx -= L; if (dx < -L / 2) dx += L;
    if (dy > L / 2) dy -= L; if (dy < -L / 2) dy += L;
    if (dz > L / 2) dz -= L; if (dz < -L / 2) dz += L;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main() {
    // Initialize agents
    std::vector<Agent> agents(N);
    std::uniform_real_distribution<double> uniform_pos(0.0, L);
    std::uniform_real_distribution<double> uniform_cos_theta(-1.0, 1.0); // for sampling polar angle correctly

    for (int i = 0; i < N; ++i) {
        // Random position in cube
        agents[i].x = uniform_pos(rng);
        agents[i].y = uniform_pos(rng);
        agents[i].z = uniform_pos(rng);

        // Generate random unit vector (vx, vy, vz) on sphere
        double phi = uniform_angle(rng);                      // azimuthal angle: [0, 2π]
        double cos_theta = uniform_cos_theta(rng);            // cos(theta): [-1, 1]
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta); // sin(theta)

        agents[i].vx = sin_theta * std::cos(phi);
        agents[i].vy = sin_theta * std::sin(phi);
        agents[i].vz = cos_theta;
    }

    std::ofstream output("vicsek_3D_output.csv");

    // Simulation loop
    for (int step = 0; step < steps; ++step) {
        // Output positions
        for (const auto& a : agents) {
            output << step << "," << a.x << "," << a.y << "," << a.z << "\n";
        }

        // Compute new directions
        std::vector<std::tuple<double, double, double>> new_velocities(N);
        for (int i = 0; i < N; ++i) {
            double sum_vx = 0.0;
            double sum_vy = 0.0;
            double sum_vz = 0.0;

            for (int j = 0; j < N; ++j) {
                if (distance(agents[i].x, agents[i].y, agents[i].z, agents[j].x, agents[j].y, agents[j].z) < R) {
                    sum_vx += agents[j].vx;
                    sum_vy += agents[j].vy;
                    sum_vz += agents[j].vz;
                }
            }

            double mag = std::sqrt(sum_vx * sum_vx + sum_vy * sum_vy + sum_vz * sum_vz);
            if (mag > 0) {
                sum_vx /= mag;
                sum_vy /= mag;
                sum_vz /= mag;
            }

            // Store in temporary agent and apply noise
            Agent temp;
            temp.vx = sum_vx;
            temp.vy = sum_vy;
            temp.vz = sum_vz;
            temp.normalize();   // Ensure it's unit length before applying noise
            temp.apply_noise(); // Applies Rodrigues' rotation

            // Store result for update
            new_velocities[i] = {temp.vx, temp.vy, temp.vz};
        }

        // Update velocities and move
        for (int i = 0; i < N; ++i) {
        agents[i].vx = std::get<0>(new_velocities[i]);
        agents[i].vy = std::get<1>(new_velocities[i]);
        agents[i].vz = std::get<2>(new_velocities[i]);
        agents[i].move();
}
    }

    output.close();
    std::cout << "Simulation complete. Output saved to vicsek_3D_output.csv.\n";

    return 0;
}
