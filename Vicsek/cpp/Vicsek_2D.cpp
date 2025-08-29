#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

// Constants
const int N = 100;
const double L = 10.0;
const double R = 1.0;
const double v0 = 0.03;
const double eta = 0.1;
const double dt = 1.0;
const int steps = 100;

// Random number generator
std::mt19937 rng(42);
std::uniform_real_distribution<double> uniform_angle(0.0, 2.0 * M_PI);
std::uniform_real_distribution<double> uniform_noise(-eta / 2.0, eta / 2.0);

// Agent with direction as a vector (vx, vy)
struct Agent {
    double x, y;      // Position
    double vx, vy;    // Direction unit vector

    // Move the agent
    void move() {
        x += v0 * vx;
        y += v0 * vy;

        // Periodic boundary conditions
        if (x < 0) x += L;
        if (x >= L) x -= L;
        if (y < 0) y += L;
        if (y >= L) y -= L;
    }

    // Normalize direction vector
    void normalize() {
        double mag = std::sqrt(vx * vx + vy * vy);
        if (mag > 0) {
            vx /= mag;
            vy /= mag;
        }
    }

    // Add random angular noise (rotation)
    void apply_noise(double noise_angle) {
        double cos_a = std::cos(noise_angle);
        double sin_a = std::sin(noise_angle);
        double new_vx = vx * cos_a - vy * sin_a;
        double new_vy = vx * sin_a + vy * cos_a;
        vx = new_vx;
        vy = new_vy;
        normalize();
    }
};

// Compute distance with periodic boundaries
double distance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;

    if (dx > L / 2) dx -= L;
    if (dx < -L / 2) dx += L;
    if (dy > L / 2) dy -= L;
    if (dy < -L / 2) dy += L;

    return std::sqrt(dx * dx + dy * dy);
}

int main() {
    // Initialize agents
    std::vector<Agent> agents(N);
    std::uniform_real_distribution<double> uniform_pos(0.0, L);
    for (int i = 0; i < N; ++i) {
        double theta = uniform_angle(rng);
        agents[i].x = uniform_pos(rng);
        agents[i].y = uniform_pos(rng);
        agents[i].vx = std::cos(theta);
        agents[i].vy = std::sin(theta);
    }

    std::ofstream output("vicsek_2D_output.csv");

    // Simulation loop
    for (int step = 0; step < steps; ++step) {
        // Output positions
        for (const auto& a : agents) {
            output << step << "," << a.x << "," << a.y << "\n";
        }

        // Compute new directions
        std::vector<std::pair<double, double>> new_velocities(N);
        for (int i = 0; i < N; ++i) {
            double sum_vx = 0.0;
            double sum_vy = 0.0;

            for (int j = 0; j < N; ++j) {
                if (distance(agents[i].x, agents[i].y, agents[j].x, agents[j].y) < R) {
                    sum_vx += agents[j].vx;
                    sum_vy += agents[j].vy;
                }
            }

            double mag = std::sqrt(sum_vx * sum_vx + sum_vy * sum_vy);
            if (mag > 0) {
                sum_vx /= mag;
                sum_vy /= mag;
            }

            // Apply noise via rotation
            double noise_angle = uniform_noise(rng);
            double cos_a = std::cos(noise_angle);
            double sin_a = std::sin(noise_angle);
            double noisy_vx = sum_vx * cos_a - sum_vy * sin_a;
            double noisy_vy = sum_vx * sin_a + sum_vy * cos_a;

            new_velocities[i] = {noisy_vx, noisy_vy};
        }

        // Update velocities and move
        for (int i = 0; i < N; ++i) {
            agents[i].vx = new_velocities[i].first;
            agents[i].vy = new_velocities[i].second;
            agents[i].move();
        }
    }

    output.close();
    std::cout << "Simulation complete. Output saved to vicsek_2D_outputs.csv.\n";

    return 0;
}
