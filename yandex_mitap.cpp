//============================================================================
// Name        : yandex_mitap.cpp
// Author      : sonulen
// Description : Hello World in C, Ansi-style
//============================================================================

#include <iostream>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <math.h>
#include <cmath> // для round

struct point_3d {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct plane_t {
    float A = 0;
    float B = 0;
    float C = 0;
    float D = 0;
};

plane_t abcd (const point_3d& p1, const point_3d& p2, const point_3d& p3) {
    plane_t plane;
    float x1 = p1.x, y1 = p1.y, z1 = p1.z;
    float x2 = p2.x, y2 = p2.y, z2 = p2.z;
    float x3 = p3.x, y3 = p3.y, z3 = p3.z;
//    auto [x1,y1,z1] = p1;
//    auto [x2,y2,z2] = p2;
//    auto [x3,y3,z3] = p3;

    plane.A = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
    plane.B = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
    plane.C = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
    plane.D = (x1 * (y2 * z3 - y3 * z2) + x2 * (y3 * z1 - y1 * z3)
            + x3 * (y1 * z2 - y2 * z1));
    if (plane.D) {
        plane.D *= -1;
    }

    return plane;
}

void find_random_points (point_3d& p1, point_3d& p2, point_3d& p3,
                         const std::vector<point_3d>& cloud) {
    int num1 = 0,num2 = 0,num3 = 0;
    size_t size = cloud.size();

    num1 = rand() % size;
    do {
        num2 = rand() % size;
    } while (num1 == num2);

    do {
        num3 = rand()% size;
    } while(num3 == num1 || num2 == num3);

    p1 = cloud[num1];
    p2 = cloud[num2];
    p3 = cloud[num3];
}

double distance_to_dot (const plane_t& plane, const point_3d& point) {
    float p = std::fabs(plane.A * point.x + plane.B * point.y +
            plane.C * point.z + plane.D);
    p /= sqrtf(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C);
    return p;
}

int calc_inliers_points (const std::vector<point_3d>& cloud,
                         const float& threshold, const plane_t& plane) {
    int result = 0;
    for (auto point : cloud) {
        if (distance_to_dot ( plane, point ) <= threshold) {
            result++;
        }
    }
    return result;
}

plane_t find_plane (const std::vector<point_3d>& cloud,
                    const float& threshold, int max_iteration,
                    const int goal_inliers) {
    plane_t plane;

    int best_goal = 0;
    plane_t best_plane;

    point_3d p1;
    point_3d p2;
    point_3d p3;

    for (; max_iteration > 0; max_iteration--) {
        find_random_points(p1, p2, p3, cloud);
        plane = abcd (p1, p2, p3);

        if (plane.A == 0 && plane.B == 0 && plane.C == 0) {
        	continue;
        }

        int ic = calc_inliers_points (cloud, threshold, plane);

        if (ic > best_goal) {
            best_goal = ic;
            best_plane = plane;
            if (best_goal > goal_inliers) {
                break;
            }
        }
    }

    // Нормализация
    float prod = 1;
    if (best_plane.D > 0) {
        prod = -1;
    }
    prod /= sqrt(best_plane.A * best_plane.A + best_plane.B * best_plane.B
                 + best_plane.C * best_plane.C);

    best_plane.A *= prod;
    best_plane.B *= prod;
    best_plane.C *= prod;
    best_plane.D *= prod;
    return best_plane;
}

float round_num (float value, int n) {
	int divider = powf(10,n);
	return (round(value * divider) / divider);
}

int main() {
    float p = 0;
    int point_count = 0;
    std::vector<point_3d> cloud;
    int max_iteration = 10000;

    std::cin >> p;
    std::cin >> point_count;

    for (int i = 0; i < point_count; i++) {
        point_3d new_point;
        std::cin >> new_point.x >> new_point.y >> new_point.z;
        cloud.push_back(new_point);
    }
    
    plane_t plane = find_plane(cloud, p, max_iteration, cloud.size()/2);

    std::cout << round_num(plane.A, 6) << " "
            << round_num(plane.B, 6) << " "
            << round_num(plane.C, 6) << " "
            << round_num(plane.D, 6) << std::endl;

    return 0;
}
