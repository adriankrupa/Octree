#include <stdio.h>
#include <glm/glm.hpp>
#include <fstream>
#include <iostream>

#include "gtest/gtest.h"
#include "Octree.h"

using namespace AKOctree2;

struct Point {
  glm::vec3 position;
  float mass;
};

unsigned int points = POINTS;
Point testPoint;


class OctreePointAgent : public OctreeAgent<Point> {

public:
    virtual bool isItemOverlappingCell(const Point *item,
                                       const OctreeVec3<float> &cellCenter,
                                       const float &cellRadius) const {

    if (glm::abs(item->position.x - cellCenter.x) > cellRadius ||
        glm::abs(item->position.y - cellCenter.y) > cellRadius ||
        glm::abs(item->position.z - cellCenter.z) > cellRadius) {
            return false;
    }
        return true;
  }
/*
    virtual glm::vec3 GetMaxValuesForAutoAdjust(const Point *item, const glm::vec3 &max) const override {
        return glm::vec3(glm::max((float)item->position.x, max.x), glm::max((float)item->position.y, max.y), glm::max((float)item->position.z, max.z));
    }

    virtual glm::vec3 GetMinValuesForAutoAdjust(const Point *item, const glm::vec3 &min) const override {
        return glm::vec3(glm::min((float)item->position.x, min.x), glm::min((float)item->position.y, min.y), glm::min((float)item->position.z, min.z));
    }
*/
};
/*


class OctreePointVisitor : public OctreeVisitor<Point> {
public:
    virtual void  visitRoot  ( const OctreeCell<Point>* rootCell) const override {
        ContinueVisit(rootCell);
        testPoint = rootCell->GetBranchData();
    }

    virtual void  visitBranch( const OctreeCell<Point>* const childs[8], Point& branchData) const override {
        branchData.mass = 0.0f;
        branchData.position = glm::vec3(0);
        for (int i = 0; i < 8; ++i) {
            ContinueVisit(childs[i]);
            branchData.mass += childs[i]->GetBranchData().mass;
            branchData.position += childs[i]->GetBranchData().position * childs[i]->GetBranchData().mass;
        }
        if (branchData.mass > 0.0f) {
            branchData.position /= branchData.mass;
        }
    }

    virtual void  visitLeaf  ( const std::vector<const Point*>& items, Point& branchData) const override {
        branchData.mass = 0.0f;
        branchData.position = glm::vec3(0);
        for (int i = 0; i < items.size(); ++i) {
            branchData.mass += items[i]->mass;
            branchData.position += items[i]->mass * items[i]->position;
        }

        if (branchData.mass > 0.0f) {
            branchData.position /= branchData.mass;
        }
    }

    virtual void visitPreRoot(const OctreeCell<Point, Point> *rootCell) const override {
    }

    virtual void visitPostRoot(const OctreeCell<Point, Point> *rootCell) const override {
        testPoint = rootCell->GetBranchData();
    }

    virtual void visitPreBranch(const OctreeCell<Point, Point> *const childs[8], Point &branchData) const override {
    }

    virtual void visitPostBranch(const OctreeCell<Point, Point> *const childs[8], Point &branchData) const override {
        branchData.mass = 0.0f;
        branchData.position = glm::vec3(0);
        for (int i = 0; i < 8; ++i) {
            branchData.mass += childs[i]->GetBranchData().mass;
            branchData.position += childs[i]->GetBranchData().position * childs[i]->GetBranchData().mass;
        }
        if (branchData.mass > 0.0f) {
            branchData.position /= branchData.mass;
        }
    }
};
*/
class OctreeTests : public ::testing::Test {

  protected:
    Octree<Point, Point> *o;
    //Octree<Point> *o2;

    virtual void SetUp() {
        Test::SetUp();
        testPoint.mass = 0;
        testPoint.position = glm::vec3(0);
    }
    virtual void TearDown() {
        Test::TearDown();
        delete o;
    }
};
/*

TEST_F (OctreeTests, GenerateData) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    Point *p = new Point[points];
    std::fstream outputFile;
    outputFile.open("denseTest.txt", std::ios::out | std::ios::binary);
    for (int i = 0; i < points; ++i) {
        p[i].position = glm::dvec3(rand()%10000/100000.0f, rand()%10000/100000.0f, rand()%10000/100000.0f);
        p[i].mass = 1.0f;
    }
    outputFile.write((char *) p, points * sizeof(Point));
    outputFile.close();

    outputFile.open("test.txt", std::ios::out | std::ios::binary);
    for (int i = 0; i < points; ++i) {
        p[i].position = glm::vec3(rand()%200000/1000.0f-100.0f, rand()%200000/1000.0f-100.0f, rand()%200000/1000.0f-100.0f);
        p[i].mass = 1.0f;
    }
    outputFile.write((char *) p, points * sizeof(Point));
    outputFile.close();
    delete p;
}
*/

TEST_F (OctreeTests, OctreeDefaultConstructor) {
  o = new Octree<Point, Point>(4);
  ASSERT_NE(o, nullptr);
}

TEST_F (OctreeTests, OctreMaxItemsPerCellGetter) {
  o = new Octree<Point, Point>(4);
  ASSERT_EQ(4, o->getMaxItemsPerCell());
}

TEST_F (OctreeTests, SingleInsertTest) {
    o = new Octree<Point, Point>(4);
    OctreePointAgent agent;
    Point *p = new Point[1];
    p[0].position = glm::vec3(1,1,1);
    o->insert(&p[0], agent);
    ASSERT_EQ("", o->getItemPath(&p[0]));

    delete p;
}


TEST_F (OctreeTests, Insert5Points) {
    o = new Octree<Point, Point>(4);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);
    ASSERT_EQ("344", o->getItemPath(&p[0]));
    ASSERT_EQ("344", o->getItemPath(&p[1]));
    ASSERT_EQ("345", o->getItemPath(&p[2]));
    ASSERT_EQ("345", o->getItemPath(&p[3]));
    ASSERT_EQ("345", o->getItemPath(&p[4]));

    delete p;
}

TEST_F (OctreeTests, Insert5PointsReverse) {
    o = new Octree<Point, Point>(4);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[4], agent);
    o->insert(&p[3], agent);
    o->insert(&p[2], agent);
    o->insert(&p[1], agent);
    o->insert(&p[0], agent);
    ASSERT_EQ("344", o->getItemPath(&p[0]));
    ASSERT_EQ("344", o->getItemPath(&p[1]));
    ASSERT_EQ("345", o->getItemPath(&p[2]));
    ASSERT_EQ("345", o->getItemPath(&p[3]));
    ASSERT_EQ("345", o->getItemPath(&p[4]));

    delete p;
}

TEST_F (OctreeTests, Insert5PointsSingleValuesInLeaves) {
    o = new Octree<Point, Point>(1);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);
    ASSERT_EQ("3444", o->getItemPath(&p[0]));
    ASSERT_EQ("3445", o->getItemPath(&p[1]));
    ASSERT_EQ("3454", o->getItemPath(&p[2]));
    ASSERT_EQ("34552", o->getItemPath(&p[3]));
    ASSERT_EQ("34553", o->getItemPath(&p[4]));

    delete p;
}
/*
TEST_F (OctreeTests, Insert5PointsAtOnce) {
    o = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(p, 5, agent);
    ASSERT_EQ("3444", o->GetItemPath(&p[0]));
    ASSERT_EQ("3445", o->GetItemPath(&p[1]));
    ASSERT_EQ("3454", o->GetItemPath(&p[2]));
    ASSERT_EQ("34552", o->GetItemPath(&p[3]));
    ASSERT_EQ("34553", o->GetItemPath(&p[4]));

    delete p;
}

TEST_F (OctreeTests, Insert5PointsAtOnceWithAdjust) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0, glm::vec3(0), 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(p, 5, agent);
    o2->insert(p, 5, agent, true);
    ASSERT_EQ("3444", o->GetItemPath(&p[0]));
    ASSERT_EQ("3445", o->GetItemPath(&p[1]));
    ASSERT_EQ("3454", o->GetItemPath(&p[2]));
    ASSERT_EQ("34552", o->GetItemPath(&p[3]));
    ASSERT_EQ("34553", o->GetItemPath(&p[4]));
    ASSERT_EQ(o->ForceGetItemsCount(), o2->ForceGetItemsCount());

    delete p;
}

TEST_F (OctreeTests, Insert5PointsAtOnceWithVector) {
    o = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    std::vector<Point> points;
    Point p;
    points.push_back(p);
    points.push_back(p);
    points.push_back(p);
    points.push_back(p);
    points.push_back(p);
    points[0].position = glm::vec3(1,1,1);
    points[1].position = glm::vec3(2,1,1);
    points[2].position = glm::vec3(3,1,1);
    points[3].position = glm::vec3(4,1,1);
    points[4].position = glm::vec3(5,1,1);
    o->insert(points, agent);
    ASSERT_EQ("3444", o->GetItemPath(&points[0]));
    ASSERT_EQ("3445", o->GetItemPath(&points[1]));
    ASSERT_EQ("3454", o->GetItemPath(&points[2]));
    ASSERT_EQ("34552", o->GetItemPath(&points[3]));
    ASSERT_EQ("34553", o->GetItemPath(&points[4]));
}

TEST_F (OctreeTests, GetItemsCountTest) {
    o = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);
    ASSERT_EQ(5, o->GetItemsCount());

    delete p;
}

TEST_F (OctreeTests, TestVisitSinglePoint) {
    o = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[1];
    p[0].position = glm::vec3(1.0f,2.0f,-1.0f);
    p[0].mass = 3.0f;
    o->insert(&p[0], agent);
    o->visit(visitor);
    ASSERT_FLOAT_EQ(3.0f, testPoint.mass);
    ASSERT_FLOAT_EQ(1.0f, testPoint.position.x);
    ASSERT_FLOAT_EQ(2.0f, testPoint.position.y);
    ASSERT_FLOAT_EQ(-1.0f, testPoint.position.z);

    delete p;
}

TEST_F (OctreeTests, TestVisit5Points) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0, glm::vec3(0), 0);
    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[0].mass = 3.0f;
    p[1].position = glm::vec3(2,5,3);
    p[1].mass = 1.0f;
    p[2].position = glm::vec3(-3,1,-1);
    p[2].mass = 10.0f;
    p[3].position = glm::vec3(4,1,5);
    p[3].mass = 2.0f;
    p[4].position = glm::vec3(-5,-10,1);
    p[4].mass = 1.0f;
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);
    o2->insert(p, 5, agent, true);

    o->visit(visitor);
    ASSERT_FLOAT_EQ(17.0f, testPoint.mass);
    ASSERT_FLOAT_EQ(-22.0f/17.0f, testPoint.position.x);
    ASSERT_FLOAT_EQ(10.0f/17.0f, testPoint.position.y);
    ASSERT_FLOAT_EQ(7.0f/17.0f, testPoint.position.z);

    testPoint.position = glm::vec3(0);
    testPoint.mass = 0.0f;

    o2->visit(visitor);
    ASSERT_FLOAT_EQ(17.0f, testPoint.mass);
    ASSERT_FLOAT_EQ(-22.0f/17.0f, testPoint.position.x);
    ASSERT_FLOAT_EQ(10.0f/17.0f, testPoint.position.y);
    ASSERT_FLOAT_EQ(7.0f/17.0f, testPoint.position.z);

    delete p;
}

TEST_F (OctreeTests, TestVisit5PointsInThreads) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0, 2);
    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[0].mass = 3.0f;
    p[1].position = glm::vec3(2,5,3);
    p[1].mass = 1.0f;
    p[2].position = glm::vec3(-3,1,-1);
    p[2].mass = 10.0f;
    p[3].position = glm::vec3(4,1,5);
    p[3].mass = 2.0f;
    p[4].position = glm::vec3(-5,-10,1);
    p[4].mass = 1.0f;

    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);

    o2->insert(&p[0], agent);
    o2->insert(&p[1], agent);
    o2->insert(&p[2], agent);
    o2->insert(&p[3], agent);
    o2->insert(&p[4], agent);

    ASSERT_TRUE(*o == *o2);

    o->visit(visitor);

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);
    o2->visit(visitor);
    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);

    delete p;
}

TEST_F (OctreeTests, EqualityOperatorTest) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[0].mass = 3.0f;
    p[1].position = glm::vec3(2,5,3);
    p[1].mass = 1.0f;
    p[2].position = glm::vec3(-3,1,-1);
    p[2].mass = 10.0f;
    p[3].position = glm::vec3(4,1,5);
    p[3].mass = 2.0f;
    p[4].position = glm::vec3(-5,-10,1);
    p[4].mass = 1.0f;
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);

    o2->insert(&p[0], agent);
    o2->insert(&p[1], agent);
    o2->insert(&p[2], agent);
    o2->insert(&p[3], agent);
    o2->insert(&p[4], agent);
    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(o->ForceGetItemsCount() == o2->ForceGetItemsCount());

    delete p;
}

TEST_F (OctreeTests, ReverseInputEqualityOperatorTest) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[0].mass = 3.0f;
    p[1].position = glm::vec3(2,5,3);
    p[1].mass = 1.0f;
    p[2].position = glm::vec3(-3,1,-1);
    p[2].mass = 10.0f;
    p[3].position = glm::vec3(4,1,5);
    p[3].mass = 2.0f;
    p[4].position = glm::vec3(-5,-10,1);
    p[4].mass = 1.0f;
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);

    o2->insert(&p[4], agent);
    o2->insert(&p[3], agent);
    o2->insert(&p[2], agent);
    o2->insert(&p[1], agent);
    o2->insert(&p[0], agent);
    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(o->ForceGetItemsCount() == o2->ForceGetItemsCount());

    delete p;
}

TEST_F (OctreeTests, InequalityOperatorTest) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[0].mass = 3.0f;
    p[1].position = glm::vec3(2,5,3);
    p[1].mass = 1.0f;
    p[2].position = glm::vec3(-3,1,-1);
    p[2].mass = 10.0f;
    p[3].position = glm::vec3(4,1,5);
    p[3].mass = 2.0f;
    p[4].position = glm::vec3(-5,-10,1);
    p[4].mass = 1.0f;
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);

    o2->insert(&p[0], agent);
    o2->insert(&p[1], agent);
    o2->insert(&p[2], agent);
    o2->insert(&p[3], agent);
    ASSERT_FALSE(*o==*o2);

    delete p;
}

TEST_F (OctreeTests, SamePointInsertTest) {
    o = new Octree<Point, Point>(1, 0, 0);
    o2 = new Octree<Point, Point>(1, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[0].mass = 3.0f;
    p[1].position = glm::vec3(2,5,3);
    p[1].mass = 1.0f;
    p[2].position = glm::vec3(-3,1,-1);
    p[2].mass = 10.0f;
    p[3].position = glm::vec3(4,1,5);
    p[3].mass = 2.0f;
    p[4].position = glm::vec3(-5,-10,1);
    p[4].mass = 1.0f;
    o->insert(&p[0], agent);
    o->insert(&p[1], agent);
    o->insert(&p[2], agent);
    o->insert(&p[3], agent);
    o->insert(&p[4], agent);

    o2->insert(&p[0], agent);
    o2->insert(&p[1], agent);
    o2->insert(&p[2], agent);
    o2->insert(&p[3], agent);
    o2->insert(&p[4], agent);
    o2->insert(&p[4], agent);
    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(o->ForceGetItemsCount() == o2->ForceGetItemsCount());

    delete p;
}

TEST_F (OctreeTests, Insert10PointsAtOnceWithThreads) {

    o = new Octree<Point, Point>(4, 0, 0, 0);
    o2 = new Octree<Point, Point>(4, 0, 0);
    OctreePointAgent agent;
    Point *p = new Point[10];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    p[5].position = glm::vec3(6,1,1);
    p[6].position = glm::vec3(7,1,1);
    p[7].position = glm::vec3(8,1,1);
    p[8].position = glm::vec3(9,1,1);
    p[9].position = glm::vec3(10,1,1);
    o->insert(p, 10, agent);
    o2->insert(p, 10, agent);

    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(o->ForceGetItemsCount() == o2->ForceGetItemsCount());

    delete p;
}

TEST_F (OctreeTests, PerformanceSparseInsertTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    OctreePointAgent agent;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();


    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_EQ(o->GetItemsCount(), o2->GetItemsCount());
    ASSERT_EQ(o->ForceGetItemsCount(), o2->ForceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    delete p;
}

TEST_F (OctreeTests, PerformanceSparseInsertAdjustTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    auto oAdjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0);

    OctreePointAgent agent;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    start2 = std::chrono::steady_clock::now();
    o2Adjust->insert(p, points, agent, true);
    end2 = std::chrono::steady_clock::now();
    diff2 = end2 - start2;
    std::cout << "1 thread with auto adjust: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    oAdjust->insert(p, points, agent, true);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;


    ASSERT_EQ(o->GetItemsCount(), o2->GetItemsCount());
    //ASSERT_EQ(oAdjust->GetItemsCount(), o2Adjust->GetItemsCount());
    ASSERT_EQ(o->ForceGetItemsCount(), o2->ForceGetItemsCount());
    ASSERT_EQ(oAdjust->ForceGetItemsCount(), o2Adjust->ForceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(*oAdjust == *o2Adjust);
    delete p;
}

TEST_F (OctreeTests, PerformanceDenseInsertTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 7);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    OctreePointAgent agent;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();


    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_EQ(o->GetItemsCount(), o2->GetItemsCount());
    ASSERT_EQ(o->ForceGetItemsCount(), o2->ForceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    delete p;
}

TEST_F (OctreeTests, PerformanceDenseInsertAdjustTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    auto oAdjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0);

    OctreePointAgent agent;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    start2 = std::chrono::steady_clock::now();
    o2Adjust->insert(p, points, agent, true);
    end2 = std::chrono::steady_clock::now();
    diff2 = end2 - start2;
    std::cout << "1 thread with auto adjust: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    oAdjust->insert(p, points, agent, true);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;


    ASSERT_EQ(o->GetItemsCount(), o2->GetItemsCount());
    //ASSERT_EQ(oAdjust->GetItemsCount(), o2Adjust->GetItemsCount());
    ASSERT_EQ(o->ForceGetItemsCount(), o2->ForceGetItemsCount());
    ASSERT_EQ(oAdjust->ForceGetItemsCount(), o2Adjust->ForceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(*oAdjust == *o2Adjust);
    delete p;
}

TEST_F (OctreeTests, PerformanceSparseVisitTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, agent);
    o->insert(p, points, agent);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    auto start = std::chrono::steady_clock::now();
    o->visit(visitor);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    delete p;
}

TEST_F (OctreeTests, PerformanceSparseVisitAdjustTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    auto oAdjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0);

    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, agent);
    o->insert(p, points, agent);

    o2Adjust->insert(p, points, agent, true);
    oAdjust->insert(p, points, agent, true);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(*oAdjust==*o2Adjust);

    auto start = std::chrono::steady_clock::now();
    o->visit(visitor);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    start2 = std::chrono::steady_clock::now();
    o2Adjust->visit(visitor);
    end2 = std::chrono::steady_clock::now();
    diff2 = end2 - start2;
    std::cout << "1 thread with auto adjust: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    start = std::chrono::steady_clock::now();
    o->visit(visitor);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);

    delete p;
}

TEST_F (OctreeTests, PerformanceDenseVisitTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, agent);
    o->insert(p, points, agent);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    auto start = std::chrono::steady_clock::now();
    o->visit(visitor);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    delete p;
}

TEST_F (OctreeTests, PerformanceDenseVisitAdjustTests) {
    o = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100, 0);
    o2 = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 100);

    auto oAdjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point>(8, 0, 0, glm::vec3(0), 0);

    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, agent);
    o->insert(p, points, agent);

    o2Adjust->insert(p, points, agent, true);
    oAdjust->insert(p, points, agent, true);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(*oAdjust==*o2Adjust);

    auto start = std::chrono::steady_clock::now();
    o->visit(visitor);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    start2 = std::chrono::steady_clock::now();
    o2Adjust->visit(visitor);
    end2 = std::chrono::steady_clock::now();
    diff2 = end2 - start2;
    std::cout << "1 thread with auto adjust: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    start = std::chrono::steady_clock::now();
    o->visit(visitor);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);

    delete p;
}
*/

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
