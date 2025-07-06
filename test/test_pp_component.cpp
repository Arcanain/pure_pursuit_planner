#include <gtest/gtest.h>
#include "dwa_planner/dwa_planner_component.hpp"

using namespace dwa_planner;

//---------------------------------------------
// テスト1: DynamicWindowApproach 基本テスト
//---------------------------------------------
TEST(TestDWAComponent, BasicNoObstacles)
{
  std::array<double, 5> x = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> model = {1.0, 0.35, 0.2, 0.5, 0.01, 0.01};
  std::array<double, 2> goal = {5.0, 0.0};
  std::array<double, 4> evalParam = {0.1, 0.08, 0.1, 3.0};
  std::vector<std::array<double, 2>> ob; // 障害物なし
  double R = 0.5;
  double robotR = 0.3;

  auto result = DWA::DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR);
  EXPECT_NEAR(result.size(), 2u, 1e-9);
  EXPECT_EQ(result[0], 0.02);
}

//---------------------------------------------
// テスト2: DynamicWindowApproach 障害物が全衝突
//---------------------------------------------
TEST(TestDWAComponent, AllObstaclesCollide)
{
  std::array<double, 5> x = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> model = {1.0, 0.35, 0.2, 0.5, 0.01, 0.01};
  std::array<double, 2> goal = {5.0, 0.0};
  std::array<double, 4> evalParam = {0.1, 0.08, 0.1, 3.0};
  std::vector<std::array<double, 2>> ob = {
    {0.0, 0.0}, {0.1, 0.1}, {-0.1, -0.1} // すべて衝突
  };
  double R = 0.5;
  double robotR = 0.3;

  auto result = DWA::DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR);
  EXPECT_NEAR(result[0], 0.0, 1e-9);
  EXPECT_NEAR(result[1], 0.0, 1e-9);
}

//---------------------------------------------
// テスト3: DynamicWindowApproach 評価データベースが空
//---------------------------------------------
TEST(TestDWAComponent, EmptyEvaluationDatabase)
{
  std::array<double, 5> x = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> model = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0}; // 動的ウィンドウが生成されない
  std::array<double, 2> goal = {5.0, 0.0};
  std::array<double, 4> evalParam = {0.1, 0.08, 0.1, 3.0};
  std::vector<std::array<double, 2>> ob;
  double R = 0.5;
  double robotR = 0.3;

  auto result = DWA::DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR);
  EXPECT_NEAR(result[0], 0.0, 1e-9);
  EXPECT_NEAR(result[1], 0.0, 1e-9);
}

//---------------------------------------------
// テスト4: dist >= 0.0 のテスト
//---------------------------------------------
TEST(TestDWAComponent, ValidDistanceEvaluation)
{
  std::array<double, 5> x = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> model = {1.0, 0.35, 0.2, 0.5, 0.01, 0.01};
  std::array<double, 2> goal = {5.0, 0.0};
  std::array<double, 4> evalParam = {0.1, 0.08, 0.1, 3.0};
  std::vector<std::array<double, 2>> ob = {
    {10.0, 10.0} // 十分離れた障害物
  };
  double R = 0.5;
  double robotR = 0.3;

  auto result = DWA::DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR);
  EXPECT_GT(result[0], 0.0); // 前進速度が正
  EXPECT_GE(result[1], 0.0); // 旋回速度はゼロ以上
}

//---------------------------------------------
// テスト5: evalDB.empty() == false のテスト
//---------------------------------------------
TEST(TestDWAComponent, NonEmptyEvaluationDatabase)
{
  std::array<double, 5> x = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> model = {1.0, 0.35, 0.2, 0.5, 0.01, 0.01};
  std::array<double, 2> goal = {5.0, 0.0};
  std::array<double, 4> evalParam = {0.1, 0.08, 0.1, 3.0};
  std::vector<std::array<double, 2>> ob; // 障害物なし
  double R = 0.5;
  double robotR = 0.3;

  auto result = DWA::DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR);
  EXPECT_EQ(result[0], 0.02); // 正しい前進速度が返ることを確認
  EXPECT_GE(result[1], 0.0); // 旋回速度がゼロ以上
}

//---------------------------------------------
// テスト6: NormalizeEval の分岐
//---------------------------------------------
TEST(TestDWAComponent, NormalizeEvalZeroValues)
{
  std::vector<std::array<double,5>> evalDB = {
    {0.1, 0.1, 0.0, 0.0, 0.0}, // heading, dist, vel がゼロのケース
    {0.2, 0.1, 1.0, 1.0, 1.0}  // heading, dist, vel が非ゼロ
  };

  DWA::NormalizeEval(evalDB);

  EXPECT_NEAR(evalDB[0][2], 0.0, 1e-9); // heading 正規化後ゼロ
  EXPECT_NEAR(evalDB[0][3], 0.0, 1e-9); // dist 正規化後ゼロ
  EXPECT_NEAR(evalDB[0][4], 0.0, 1e-9); // vel 正規化後ゼロ
}

// //---------------------------------------------
// // テスト7: CalcDynamicWindow のエッジケース
// //---------------------------------------------
// TEST(TestDWAComponent, CalcDynamicWindowEdgeCases)
// {
//   std::array<double,5> x = {0.0, 0.0, 0.0, 0.1, 0.1}; // 初速度が小さい
//   std::array<double,6> model = {1.0, 0.5, 0.2, 0.2, 0.01, 0.01};

//   auto Vr = DWA::CalcDynamicWindow(x, model);

//   EXPECT_NEAR(Vr[0], 0.0, 1e-9); // 最小速度がゼロ以上
//   EXPECT_NEAR(Vr[1], 0.3, 1e-9); // 最大速度が正
//   EXPECT_NEAR(Vr[2], -0.1, 1e-9); // 最小回転速度が負
//   EXPECT_NEAR(Vr[3], 0.3, 1e-9); // 最大回転速度が正
// }

//---------------------------------------------
// テスト8: 特殊ケース（障害物が非常に近い場合）
//---------------------------------------------
TEST(TestDWAComponent, CloseObstacleAvoidance)
{
  std::array<double, 5> x = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> model = {1.0, 0.35, 0.2, 0.5, 0.01, 0.01};
  std::array<double, 2> goal = {5.0, 0.0};
  std::array<double, 4> evalParam = {0.1, 0.08, 0.1, 3.0};
  std::vector<std::array<double, 2>> ob = {
    {0.1, 0.0} // 非常に近い障害物
  };
  double R = 0.5;
  double robotR = 0.3;

  auto result = DWA::DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR);
  EXPECT_NEAR(result[0], 0.0, 1e-9); // 前進速度がゼロ
  EXPECT_NEAR(result[1], 0.0, 1e-9); // 回転速度もゼロ
}
