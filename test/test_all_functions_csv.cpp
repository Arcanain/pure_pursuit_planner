#include <gtest/gtest.h>
#include "test_utils.hpp"
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

using namespace pure_pursuit_planner;
using test_utils::FuncTest;
using test_utils::runCsvTests;

const double tol = 1e-3;

TEST(computeVelocity, CsvBasedTest) {
    std::vector<FuncTest> tests = {
        FuncTest{
            "computeVelocity",
            {
                "pose_x", "pose_y", "pose_yaw", "velocity",
                "cx0", "cy0", "cyaw0", "ck0",
                "cx1", "cy1", "cyaw1", "ck1",
                "cx2", "cy2", "cyaw2", "ck2",
                "minVelocity", "maxVelocity", "minCurvature", "maxCurvature",
                "Lfc", "k"
            },
            {"exp_v", "exp_w"},
            [](const auto &r){
                // パスの設定
                std::vector<double> cx = {r.at("cx0"), r.at("cx1"), r.at("cx2")};
                std::vector<double> cy = {r.at("cy0"), r.at("cy1"), r.at("cy2")};
                std::vector<double> cyaw = {r.at("cyaw0"), r.at("cyaw1"), r.at("cyaw2")};
                std::vector<double> ck = {r.at("ck0"), r.at("ck1"), r.at("ck2")};

                // パラメータ設定
                PurePursuitConfig config;
                config.minVelocity = r.at("minVelocity");
                config.maxVelocity = r.at("maxVelocity");
                config.minCurvature = r.at("minCurvature");
                config.maxCurvature = r.at("maxCurvature");
                config.Lfc = r.at("Lfc");
                config.k = r.at("k");

                PurePursuitComponent pp(config);
                pp.setPath(cx, cy, cyaw, ck);

                Pose2D pose{r.at("pose_x"), r.at("pose_y"), r.at("pose_yaw")};
                pp.setPose(pose, r.at("velocity"));

                //auto [v, w] = pp.computeVelocity();
                //return {v, w};
                auto [v, w] = pp.computeVelocity();
                return std::vector<double>{v, w};
                
            }
        }
    };

    runCsvTests(tests, tol);
}

TEST(calcLf, CsvBasedTest) {
    std::vector<FuncTest> tests = {
        FuncTest{
            "calcLf",
            {"velocity", "k", "Lfc"},
            {"expected_Lf"},
            [](const auto &r){
                PurePursuitConfig config;
                config.k = r.at("k");
                config.Lfc = r.at("Lfc");
                PurePursuitComponent pp(config);
                double Lf = pp.calcLf(r.at("k"), r.at("velocity"), r.at("Lfc"));
                return std::vector<double>{Lf};
            }
        }
    };

    runCsvTests(tests, tol);
}

TEST(calcFirstNearestPointIndex, CsvBasedTest) {
    std::vector<FuncTest> tests = {
        FuncTest{
            "calcFirstNearestPointIndex",
            {
                "pose_x", "pose_y",
                "cx0", "cy0", "cx1", "cy1", "cx2", "cy2"
            },
            {"expected_index"},
            [](const auto &r){
                // 入力パスを構成
                std::vector<double> cx = {r.at("cx0"), r.at("cx1"), r.at("cx2")};
                std::vector<double> cy = {r.at("cy0"), r.at("cy1"), r.at("cy2")};

                // PurePursuitComponent を構成（configは仮でOK）
                PurePursuitConfig config;
                PurePursuitComponent pp(config);
                pp.setPath(cx, cy, {}, {});

                // ロボットの現在位置を設定
                Pose2D pose{r.at("pose_x"), r.at("pose_y"), 0.0};
                pp.setPose(pose, 0.0);

                int index = pp.calcFirstNearestPointIndex();
                return std::vector<double>{static_cast<double>(index)};
            }
        }
    };

    runCsvTests(tests, tol);  // index比較なので誤差なしでOK
}

TEST(calcOldNearestPointIndex, CsvBasedTest) {
    std::vector<FuncTest> tests = {
        FuncTest{
            "calcOldNearestPointIndex",
            {
                "pose_x", "pose_y", "old_index",
                "cx0", "cy0", "cx1", "cy1",
                "cx2", "cy2", "cx3", "cy3", "cx4", "cy4"
            },
            {"expected_index"},
            [](const auto &r){
                // 経路の設定
                std::vector<double> cx = {
                    r.at("cx0"), r.at("cx1"), r.at("cx2"),
                    r.at("cx3"), r.at("cx4")
                };
                std::vector<double> cy = {
                    r.at("cy0"), r.at("cy1"), r.at("cy2"),
                    r.at("cy3"), r.at("cy4")
                };

                // コンフィグ設定（仮のデフォルトでOK）
                PurePursuitConfig config;
                PurePursuitComponent pp(config);
                pp.setPath(cx, cy, {}, {});

                // 現在の姿勢と oldNearestPointIndex を設定
                Pose2D pose{r.at("pose_x"), r.at("pose_y"), 0.0};
                pp.setPose(pose, 0.0);
                pp.oldNearestPointIndex = static_cast<int>(r.at("old_index"));

                int index = pp.calcOldNearestPointIndex();
                return std::vector<double>{static_cast<double>(index)};
            }
        }
    };

    runCsvTests(tests, 0.0);  // index比較なので誤差なしでOK
}
