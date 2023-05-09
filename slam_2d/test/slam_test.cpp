// slam_test.cpp

#include <gtest/gtest.h>
#include <vector>

#include "slam_2d/occupancy_grid.hpp"

TEST(slam_2d, distance_field_test)
{
    slam_2d::OccupancyGrid grid(1, 10, 10, Eigen::Vector3d {0,0,0});

    std::vector<std::vector<int>> obstacle_field;
    obstacle_field.push_back(std::vector<int> {0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    obstacle_field.push_back(std::vector<int> {0, 0, 1, 0, 0, 0, 0, 1, 0, 0});
    obstacle_field.push_back(std::vector<int> {0, 0, 0, 1, 0, 0, 1, 0, 0, 0});
    obstacle_field.push_back(std::vector<int> {0, 0, 1, 1, 1, 1, 1, 1, 0, 0});
    obstacle_field.push_back(std::vector<int> {0, 1, 1, 0, 1, 1, 0, 1, 1, 0});
    obstacle_field.push_back(std::vector<int> {1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
    obstacle_field.push_back(std::vector<int> {1, 0, 1, 1, 1, 1, 1, 1, 0, 1});
    obstacle_field.push_back(std::vector<int> {1, 0, 1, 0, 0, 0, 0, 1, 0, 1});
    obstacle_field.push_back(std::vector<int> {0, 0, 0, 1, 1, 1, 1, 0, 0, 0});
    obstacle_field.push_back(std::vector<int> {0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    for (size_t i = 0; i < obstacle_field.size(); ++i) {
        for (size_t j = 0; j < obstacle_field[i].size(); ++j) {
            if (obstacle_field[i][j] == 1)
                grid[ (int) (i + j * 10) ].log_odds = 1;
            else
                grid[ (int) (i + j * 10) ].log_odds = -1;
        }
    }

    grid.update_distance_field();

    std::vector<std::vector<int>> square_dist_truths;
    square_dist_truths.push_back(std::vector<int> {5, 2, 1, 2, 5, 5, 2, 1, 2, 5});
    square_dist_truths.push_back(std::vector<int> {4, 1, 0, 1, 2, 2, 1, 0, 1, 4});
    square_dist_truths.push_back(std::vector<int> {5, 2, 1, 0, 1, 1, 0, 1, 2, 5});
    square_dist_truths.push_back(std::vector<int> {2, 1, 0, 0, 0, 0, 0, 0, 1, 2});
    square_dist_truths.push_back(std::vector<int> {1, 0, 0, 1, 0, 0, 1, 0, 0, 1});
    square_dist_truths.push_back(std::vector<int> {0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    square_dist_truths.push_back(std::vector<int> {0, 1, 0, 0, 0, 0, 0, 0, 1, 0});
    square_dist_truths.push_back(std::vector<int> {0, 1, 0, 1, 1, 1, 1, 0, 1, 0});
    square_dist_truths.push_back(std::vector<int> {1, 2, 1, 0, 0, 0, 0, 1, 2, 1});
    square_dist_truths.push_back(std::vector<int> {4, 5, 2, 1, 1, 1, 1, 2, 5, 4});

    for (size_t i = 0; i < square_dist_truths.size(); ++i) {
        for (size_t j = 0; j < square_dist_truths[i].size(); ++j) {
            ASSERT_EQ(grid.sedt[i][j], square_dist_truths[i][j]);
        }
    }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
