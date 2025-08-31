#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

// Constants
const int N = 1000;
const double L = 10.0;
const double R = 0.1;
const double v0 = 0.03;
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
    double r = L / 2.0;
    double angle = v0 * dt / r;

    double cos_theta = std::cos(angle);
    double sin_theta = std::sin(angle);

    double dot = x * vx + y * vy + z * vz;

    double new_x = x * cos_theta + (vy * z - vz * y) * sin_theta + vx * dot * (1 - cos_theta);
    double new_y = y * cos_theta + (vz * x - vx * z) * sin_theta + vy * dot * (1 - cos_theta);
    double new_z = z * cos_theta + (vx * y - vy * x) * sin_theta + vz * dot * (1 - cos_theta);

    // Renormalize to ensure it stays on the sphere surface
    double mag = std::sqrt(new_x * new_x + new_y * new_y + new_z * new_z);

    x = r * new_x / mag;
    y = r * new_y / mag;
    z = r * new_z / mag;
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


double spherical_distance(double x1, double y1, double z1,
                          double x2, double y2, double z2) {
    double dot = x1 * x2 + y1 * y2 + z1 * z2;
    double r = L / 2.0;
    double cos_theta = dot / (r * r);

    // Clamp for numerical stability
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    return r * std::acos(cos_theta);  // arc length on the sphere
}

int main() {
    // Initialize agents

    std::vector<Agent> agents(N);
    std::uniform_real_distribution<double> uniform_cos_theta(-1.0, 1.0); // for sampling polar angle correctly

    double r = L / 2; // define a radius for the sphere
    for (int i = 0; i < N; ++i) {

        double phi = uniform_angle(rng);      // [0, 2π]
        double cos_theta = uniform_cos_theta(rng); // [-1, 1]
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

        // Random position on the surface of a sphere
        agents[i].x = r * sin_theta * std::cos(phi);
        agents[i].y = r * sin_theta * std::sin(phi);
        agents[i].z = r * cos_theta;

        // Generate random unit vector (vx, vy, vz) on sphere 
        double phi_v = uniform_angle(rng);
        double cos_theta_v = uniform_cos_theta(rng);
        double sin_theta_v = std::sqrt(1 - cos_theta_v * cos_theta_v);

        double rx = sin_theta_v * std::cos(phi_v);
        double ry = sin_theta_v * std::sin(phi_v);
        double rz = cos_theta_v;

        // project onto perpendicular plane 
        double dot = rx * agents[i].x + ry * agents[i].y + rz * agents[i].z;
        double norm2 = agents[i].x * agents[i].x + agents[i].y * agents[i].y + agents[i].z * agents[i].z;

        agents[i].vx = rx - (dot / norm2) * agents[i].x;
        agents[i].vy = ry - (dot / norm2) * agents[i].y;
        agents[i].vz = rz - (dot / norm2) * agents[i].z;

        agents[i].normalize();
    }

    std::ofstream output("vicsek_3Dsphericalshell_output.csv");

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
                if (spherical_distance(agents[i].x, agents[i].y, agents[i].z, agents[j].x, agents[j].y, agents[j].z) < R) {
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

            double px = agents[i].x;
            double py = agents[i].y;
            double pz = agents[i].z;

            double dot = temp.vx * px + temp.vy * py + temp.vz * pz;
            double norm2 = px * px + py * py + pz * pz;

            temp.vx = temp.vx - (dot / norm2) * px;
            temp.vy = temp.vy - (dot / norm2) * py;
            temp.vz = temp.vz - (dot / norm2) * pz;
            temp.normalize();

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
    std::cout << "Simulation complete. Output saved to vicsek_3Dsphericalshell_output.csv.\n";

    return 0;
}
