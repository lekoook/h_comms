#include <gtest/gtest.h>
#include <exchange_graphs/exchange_graphs.h>

geometry_msgs::Point p000(0, 0, 0);
geometry_msgs::Point p100(1, 0, 0);
geometry_msgs::Point p010(0, 1, 0);
graph_msgs::GeometryGraph p000p100(p000, p100);
graph_msgs::GeometryGraph p000p010(p000, p010);

TEST(ExchangeGraphs, basic) {
    ExchangeGraphs eg0(0, {1, 2});
    eg0.recordNewGraph(p000p100);

    ExchangeGraphs eg1(1, {0, 2});
    eg1.recordNewGraph(p000p010);

    eg0.teach(1);

    ASSERT_EQ(eg0.graphToUnawareRobots[p000p100] == {2,});
    ASSERT_EQ(eg0.robotToUnknownGraphs[1] == {});
    ASSERT_EQ(eg0.robotToUnknownGraphs[2] == {p000p100,});

    ASSERT_EQ(eg1.robotToUnknownGraphs[0] == {p010,});
    // TODO ASSERT_EQ(eg1.graphToUnawareRobots[p000p100] == {p010,});
}
