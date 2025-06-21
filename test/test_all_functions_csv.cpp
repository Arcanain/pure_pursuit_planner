#include <gtest/gtest.h>
#include "test_utils.hpp"
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

using namespace pure_pursuit_planner;
using test_utils::FuncTest;
using test_utils::runCsvTests;

const double tol = 1e-3;

//pp-00 現在地と目標経路を入力に対して、目標経路を追従するような速度、加速度を出力
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
                //pp.setPath(cx, cy, cyaw, ck);

                Pose2D pose{r.at("pose_x"), r.at("pose_y"), r.at("pose_yaw")};
                //pp.setPose(pose, r.at("velocity"));

                //auto [v, w] = pp.computeVelocity();
                //return {v, w};
                auto [v, w] = pp.computeVelocity(cx, cy, cyaw, ck, pose, r.at("velocity"));
                return std::vector<double>{v, w};
                
            }
        }
    };

    runCsvTests(tests, tol);
}

//pp-03 ロボットの現在速度 v に比例した前方注視距離 Lf を求める
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

//pp-04 初回実行時、ロボットの現在位置に最も近い軌道上の点（最近傍点）を探索し、oldNearestPointIndex に保存
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

//pp-05 前回の注視点を元に探索範囲を限定する
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

//pp-06 現在位置から前方注視距離 Lf 以上の注視点を探す.
TEST(searchTargetIndex, CsvBasedTest) {
    std::vector<FuncTest> tests = {
        FuncTest{
            "searchTargetIndex",
            {
                "pose_x", "pose_y", "velocity",
                "k", "Lfc",
                "cx0", "cy0", "cx1", "cy1", "cx2", "cy2", "cx3", "cy3"
            },
            {"expected_index"},
            [](const auto &r){
                // パス設定
                std::vector<double> cx = {r.at("cx0"), r.at("cx1"), r.at("cx2"), r.at("cx3")};
                std::vector<double> cy = {r.at("cy0"), r.at("cy1"), r.at("cy2"), r.at("cy3")};

                // コンフィグ
                PurePursuitConfig config;
                config.k = r.at("k");
                config.Lfc = r.at("Lfc");

                // インスタンス作成
                PurePursuitComponent pp(config);
                pp.setPath(cx, cy, {}, {});
                pp.setPose(Pose2D{r.at("pose_x"), r.at("pose_y"), 0.0}, r.at("velocity"));

                // テスト対象呼び出し
                auto [index, Lf] = pp.searchTargetIndex();

                return std::vector<double>{static_cast<double>(index)};
            }
        }
    };

    runCsvTests(tests, 0.0);  // index比較のため誤差なし
}

//pp-07 曲率が大きいほど減速し、直線時（曲率 ≒ 0）には最大速度に近づくような目標速度を求める。
TEST(curvatureToVelocity, CsvBasedTest) {
    std::vector<FuncTest> tests = {
        FuncTest{
            "curvatureToVelocity",
            {"curvature", "minVelocity", "maxVelocity"},
            {"expected_velocity"},
            [](const auto &r){
                PurePursuitConfig config;
                config.minVelocity = r.at("minVelocity");
                config.maxVelocity = r.at("maxVelocity");

                PurePursuitComponent pp(config);
                double velocity = pp.curvatureToVelocity(r.at("curvature"));

                return std::vector<double>{velocity};
            }
        }
    };

    runCsvTests(tests, tol);
}

