#include <stdio.h>
#include <glm/glm.hpp>
#include <fstream>

#include "gtest/gtest.h"
#include "Octree.h"

using namespace AKOctree;

struct Point {
  glm::dvec3 position;
  double mass;
};

unsigned int points = POINTS;
Point testPoint;

class OctreePointAgent : public OctreeAgent<Point, Point, double> {

public:
    virtual bool isItemOverlappingCell(const Point *item,
                                       const OctreeVec3<double> &cellCenter,
                                       const double &cellRadius) const override {

    if (glm::abs(item->position.x - cellCenter.x) > cellRadius ||
        glm::abs(item->position.y - cellCenter.y) > cellRadius ||
        glm::abs(item->position.z - cellCenter.z) > cellRadius) {
            return false;
    }
        return true;
  }
};

class OctreePointAgentAdjust : public OctreeAgent<Point, Point, double>, public OctreeAgentAutoAdjustExtension<Point, Point, double> {

public:
    virtual bool isItemOverlappingCell(const Point *item,
                                       const OctreeVec3<double> &cellCenter,
                                       const double &cellRadius) const override {

        if (glm::abs(item->position.x - cellCenter.x) > cellRadius ||
                glm::abs(item->position.y - cellCenter.y) > cellRadius ||
                glm::abs(item->position.z - cellCenter.z) > cellRadius) {
            return false;
        }
        return true;
    }

    virtual OctreeVec3<double> GetMaxValuesForAutoAdjust(const Point *item,
                                                        const OctreeVec3<double> &max) const override {
        return OctreeVec3<double>(glm::max(item->position.x, max.x), glm::max(item->position.y, max.y), glm::max(item->position.z, max.z));
    }

    virtual OctreeVec3<double> GetMinValuesForAutoAdjust(const Point *item,
                                                            const OctreeVec3<double> &min) const override {
        return OctreeVec3<double>(glm::min(item->position.x, min.x), glm::min(item->position.y, min.y), glm::min(item->position.z, min.z));
    }
};

class OctreePointVisitor : public OctreeVisitor<Point, Point, double> {
public:
    virtual void visitRoot(const std::shared_ptr<OctreeCell<Point, Point, double> > rootCell) const override {
        ContinueVisit(rootCell);
        testPoint = rootCell->getNodeData();
    }

    virtual void visitBranch(const std::shared_ptr<OctreeCell<Point, Point, double> >  childs[8], Point &nodeData) const override {
        nodeData.mass = 0.0f;
        nodeData.position = glm::vec3(0);
        for (int i = 0; i < 8; ++i) {
            ContinueVisit(childs[i]);
            nodeData.mass += childs[i]->getNodeData().mass;
            nodeData.position += childs[i]->getNodeData().position * childs[i]->getNodeData().mass;
        }
        if (nodeData.mass > 0.0f) {
            nodeData.position /= nodeData.mass;
        }
    }

    virtual void  visitLeaf  ( const std::vector<const Point*>& items, Point& nodeData) const override {
        nodeData.mass = 0.0f;
        nodeData.position = glm::vec3(0);
        for (unsigned int i = 0; i < items.size(); ++i) {
            nodeData.mass += items[i]->mass;
            nodeData.position += items[i]->mass * items[i]->position;
        }

        if (nodeData.mass > 0.0f) {
            nodeData.position /= nodeData.mass;
        }
    }
};

class OctreePointVisitorThreaded : public OctreeVisitorThreaded<Point, Point, double> {

    virtual void visitPreBranch(const std::shared_ptr<OctreeCell<Point, Point, double> > childs[8], Point &nodeData) const override {
        nodeData.mass = 0.0f;
        nodeData.position = glm::vec3(0);
    }

    virtual void visitPostRoot(const std::shared_ptr<OctreeCell<Point, Point, double> > rootCell) const override {
        testPoint = rootCell->getNodeData();
    }

    virtual void visitPostBranch(const std::shared_ptr<OctreeCell<Point, Point, double> > childs[8], Point &nodeData) const override {
        for (int i = 0; i < 8; ++i) {
            nodeData.mass += childs[i]->getNodeData().mass;
            nodeData.position += childs[i]->getNodeData().position * childs[i]->getNodeData().mass;
        }
        if (nodeData.mass > 0.0f) {
            nodeData.position /= nodeData.mass;
        }
    }

    virtual void visitLeaf(const std::vector<const Point *> &items, Point &nodeData) const override {
        nodeData.mass = 0.0f;
        nodeData.position = glm::vec3(0);
        for (unsigned int i = 0; i < items.size(); ++i) {
            nodeData.mass += items[i]->mass;
            nodeData.position += items[i]->mass * items[i]->position;
        }

        if (nodeData.mass > 0.0f) {
            nodeData.position /= nodeData.mass;
        }
    }
};

class OctreePointPrinter : public OctreeNodeDataPrinter<Point, Point, double> {
public:
    virtual std::string GetDataString(Point& nodeData) const override {
        std::string s;

        s = " " + std::to_string((int)nodeData.mass) + " ";

        return s;
    }
};

class OctreeTests : public ::testing::Test {

  protected:
    Octree<Point, Point, double> *o = nullptr;
    Octree<Point, Point, double> *o2 = nullptr;
    //Octree<Point, Point, double> *o2;

    virtual void SetUp() {
        Test::SetUp();
        testPoint.mass = 0;
        testPoint.position = glm::vec3(0);
    }
    virtual void TearDown() {
        Test::TearDown();
        delete o;
        delete o2;
    }
};

TEST_F (OctreeTests, GenerateData) {
    Point *p = new Point[points];
    std::fstream outputFile;
    outputFile.open("denseTest.txt", std::ios::out | std::ios::binary);
    for (unsigned int i = 0; i < points; ++i) {
        p[i].position = glm::dvec3(rand()%10000/100000.0f, rand()%10000/100000.0f, rand()%10000/100000.0f);
        p[i].mass = 1.0f;
    }
    outputFile.write((char *) p, points * sizeof(Point));
    outputFile.close();

    outputFile.open("test.txt", std::ios::out | std::ios::binary);
    for (unsigned int i = 0; i < points; ++i) {
        p[i].position = glm::vec3(rand()%200000/1000.0f-100.0f, rand()%200000/1000.0f-100.0f, rand()%200000/1000.0f-100.0f);
        p[i].mass = 1.0f;
    }
    outputFile.write((char *) p, points * sizeof(Point));
    outputFile.close();
    delete []p;
}

TEST_F (OctreeTests, OctreeDefaultConstructor) {
  o = new Octree<Point, Point, double>(4);
  ASSERT_NE(o, nullptr);
}

TEST_F (OctreeTests, OctreMaxItemsPerCellGetter) {
  o = new Octree<Point, Point, double>(4);
  ASSERT_EQ(4, o->getMaxItemsPerCell());
}


TEST_F (OctreeTests, SingleInsertTest) {
    o = new Octree<Point, Point, double>(4);
    OctreePointAgent agent;
    Point *p = new Point[1];
    p[0].position = glm::vec3(1,1,1);
    o->insert(&p[0], &agent);
    ASSERT_EQ("", o->getItemPath(&p[0]));
    ASSERT_EQ(1, o->getItemsCount());
    delete []p;
}

TEST_F (OctreeTests, Insert5Points) {
    o = new Octree<Point, Point, double>(4);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);
    ASSERT_EQ("344", o->getItemPath(&p[0]));
    ASSERT_EQ("344", o->getItemPath(&p[1]));
    ASSERT_EQ("345", o->getItemPath(&p[2]));
    ASSERT_EQ("345", o->getItemPath(&p[3]));
    ASSERT_EQ("345", o->getItemPath(&p[4]));

    delete []p;
}

TEST_F (OctreeTests, GetItemsCountTest) {
    o = new Octree<Point, Point, double>(1);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);
    ASSERT_EQ(5, o->getItemsCount());

    delete []p;
}

TEST_F (OctreeTests, Insert5PointsReverse) {
    o = new Octree<Point, Point, double>(4);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[4], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[0], &agent);
    ASSERT_EQ("344", o->getItemPath(&p[0]));
    ASSERT_EQ("344", o->getItemPath(&p[1]));
    ASSERT_EQ("345", o->getItemPath(&p[2]));
    ASSERT_EQ("345", o->getItemPath(&p[3]));
    ASSERT_EQ("345", o->getItemPath(&p[4]));

    delete []p;
}

TEST_F (OctreeTests, Insert5PointsSingleValuesInLeaves) {
    o = new Octree<Point, Point, double>(1);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);
    ASSERT_EQ("3444", o->getItemPath(&p[0]));
    ASSERT_EQ("3445", o->getItemPath(&p[1]));
    ASSERT_EQ("3454", o->getItemPath(&p[2]));
    ASSERT_EQ("34552", o->getItemPath(&p[3]));
    ASSERT_EQ("34553", o->getItemPath(&p[4]));

    delete []p;
}

TEST_F (OctreeTests, Insert5PointsAtOnce) {
    o = new Octree<Point, Point, double>(1);
    OctreePointAgent agent;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(p, 5, &agent);
    ASSERT_EQ("3444", o->getItemPath(&p[0]));
    ASSERT_EQ("3445", o->getItemPath(&p[1]));
    ASSERT_EQ("3454", o->getItemPath(&p[2]));
    ASSERT_EQ("34552", o->getItemPath(&p[3]));
    ASSERT_EQ("34553", o->getItemPath(&p[4]));

    delete []p;
}

TEST_F (OctreeTests, Insert5PointsAtOnceWithAdjust) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1, OctreeVec3<double>(0),1);
    OctreePointAgent agent;
    OctreePointAgentAdjust agentAdjust;
    Point *p = new Point[5];
    p[0].position = glm::vec3(1,1,1);
    p[1].position = glm::vec3(2,1,1);
    p[2].position = glm::vec3(3,1,1);
    p[3].position = glm::vec3(4,1,1);
    p[4].position = glm::vec3(5,1,1);
    o->insert(p, 5, &agent);
    o2->insert(p, 5, &agentAdjust);
    ASSERT_EQ("3444", o->getItemPath(&p[0]));
    ASSERT_EQ("3445", o->getItemPath(&p[1]));
    ASSERT_EQ("3454", o->getItemPath(&p[2]));
    ASSERT_EQ("34552", o->getItemPath(&p[3]));
    ASSERT_EQ("34553", o->getItemPath(&p[4]));
    ASSERT_EQ(5, o->getItemsCount());
    ASSERT_EQ(o->forceGetItemsCount(), o2->forceGetItemsCount());

    delete []p;
}

TEST_F (OctreeTests, Insert5PointsAtOnceWithVector) {
    o = new Octree<Point, Point, double>(1);
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
    o->insert(points, &agent);
    ASSERT_EQ("3444", o->getItemPath(&points[0]));
    ASSERT_EQ("3445", o->getItemPath(&points[1]));
    ASSERT_EQ("3454", o->getItemPath(&points[2]));
    ASSERT_EQ("34552", o->getItemPath(&points[3]));
    ASSERT_EQ("34553", o->getItemPath(&points[4]));
}

TEST_F (OctreeTests, TestVisitSinglePoint) {
    o = new Octree<Point, Point, double>(1);
    OctreePointAgent agent;
    OctreePointVisitor visitor;
    Point *p = new Point[1];
    p[0].position = glm::vec3(1.0f,2.0f,-1.0f);
    p[0].mass = 3.0f;
    o->insert(&p[0], &agent);
    o->visit(&visitor);
    ASSERT_FLOAT_EQ(3.0f, testPoint.mass);
    ASSERT_FLOAT_EQ(1.0f, testPoint.position.x);
    ASSERT_FLOAT_EQ(2.0f, testPoint.position.y);
    ASSERT_FLOAT_EQ(-1.0f, testPoint.position.z);

    delete []p;
}

TEST_F (OctreeTests, TestVisit5Points) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1, OctreeVec3<double>(0),1);
    OctreePointAgent agent;
    OctreePointAgentAdjust agentAdjust;
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

    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);

    o2->insert(p, 5, &agentAdjust, true);

    o->visit(&visitor);
    ASSERT_FLOAT_EQ(17.0f, testPoint.mass);
    ASSERT_FLOAT_EQ(-22.0f/17.0f, testPoint.position.x);
    ASSERT_FLOAT_EQ(10.0f/17.0f, testPoint.position.y);
    ASSERT_FLOAT_EQ(7.0f/17.0f, testPoint.position.z);

    testPoint.position = glm::vec3(0);
    testPoint.mass = 0.0f;

    o2->visit(&visitor);
    ASSERT_FLOAT_EQ(17.0f, testPoint.mass);
    ASSERT_FLOAT_EQ(-22.0f/17.0f, testPoint.position.x);
    ASSERT_FLOAT_EQ(10.0f/17.0f, testPoint.position.y);
    ASSERT_FLOAT_EQ(7.0f/17.0f, testPoint.position.z);

    delete []p;
}

TEST_F (OctreeTests, EqualityOperatorTest) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1);
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
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);

    o2->insert(&p[0], &agent);
    o2->insert(&p[1], &agent);
    o2->insert(&p[2], &agent);
    o2->insert(&p[3], &agent);
    o2->insert(&p[4], &agent);
    ASSERT_FALSE(o==o2);
    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(o->getItemsCount() == o2->getItemsCount());
    ASSERT_TRUE(o->forceGetItemsCount() == o2->forceGetItemsCount());

    delete []p;
}

TEST_F (OctreeTests, ReverseInputEqualityOperatorTest) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1);
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
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);

    o2->insert(&p[4], &agent);
    o2->insert(&p[3], &agent);
    o2->insert(&p[2], &agent);
    o2->insert(&p[1], &agent);
    o2->insert(&p[0], &agent);
    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(o->forceGetItemsCount() == o2->forceGetItemsCount());

    delete []p;
}

TEST_F (OctreeTests, InequalityOperatorTest) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1);
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
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);

    o2->insert(&p[0], &agent);
    o2->insert(&p[1], &agent);
    o2->insert(&p[2], &agent);
    o2->insert(&p[3], &agent);
    ASSERT_TRUE(*o != *o2);

    delete []p;
}

TEST_F (OctreeTests, SamePointInsertTest) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1);
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
    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);

    o2->insert(&p[0], &agent);
    o2->insert(&p[1], &agent);
    o2->insert(&p[2], &agent);
    o2->insert(&p[3], &agent);
    o2->insert(&p[4], &agent);
    o2->insert(&p[4], &agent);
    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(o->forceGetItemsCount() == o2->forceGetItemsCount());

    delete []p;
}

TEST_F (OctreeTests, Insert10PointsAtOnceWithThreads) {
    o = new Octree<Point, Point, double>(4, 0);
    o2 = new Octree<Point, Point, double>(4);
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
    o->insert(p, 10, &agent);
    o2->insert(p, 10, &agent);

    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(o->getItemsCount() == o2->getItemsCount());
    ASSERT_TRUE(o->forceGetItemsCount() == o2->forceGetItemsCount());

    delete []p;
}

TEST_F (OctreeTests, Insert10PointsAtOnceWithThreadsWithVector) {
    o = new Octree<Point, Point, double>(4, 0);
    o2 = new Octree<Point, Point, double>(4);
    OctreePointAgent agent;
    std::vector<Point> points;
    Point p;
    points.push_back(p);
    points.push_back(p);
    points.push_back(p);
    points.push_back(p);
    points.push_back(p);
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
    points[5].position = glm::vec3(6,1,1);
    points[6].position = glm::vec3(7,1,1);
    points[7].position = glm::vec3(8,1,1);
    points[8].position = glm::vec3(9,1,1);
    points[9].position = glm::vec3(10,1,1);

    o->insert(points, &agent);
    o2->insert(points, &agent);

    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(o->getItemsCount() == o2->getItemsCount());
    ASSERT_TRUE(o->forceGetItemsCount() == o2->forceGetItemsCount());
}

TEST_F (OctreeTests, LockTest) {
    o = new Octree<Point, Point, double>(4, 0);
    OctreePointAgent agent;
    std::vector<Point> points;
    Point p;
    for (int i = 0; i < 30; ++i) {
        p.position = glm::vec3(0.2 + 0.005*i, 1, 1);
        points.push_back(p);
    }
    o->insert(points, &agent);
    ASSERT_EQ(30, o->forceGetItemsCount());

}

TEST_F (OctreeTests, TestVisit5PointsInThreads) {
    o = new Octree<Point, Point, double>(1);
    o2 = new Octree<Point, Point, double>(1, 0);
    OctreePointAgent agent;
    OctreePointVisitor visitor;
    OctreePointVisitorThreaded visitorThreaded;
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

    o->insert(&p[0], &agent);
    o->insert(&p[1], &agent);
    o->insert(&p[2], &agent);
    o->insert(&p[3], &agent);
    o->insert(&p[4], &agent);

    o2->insert(&p[0], &agent);
    o2->insert(&p[1], &agent);
    o2->insert(&p[2], &agent);
    o2->insert(&p[3], &agent);
    o2->insert(&p[4], &agent);

    ASSERT_TRUE(*o == *o2);

    o->visit(&visitor);

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);
    o2->visit(&visitorThreaded);
    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);

    delete []p;
}

TEST_F (OctreeTests, PerformanceSparseInsertTests) {
    printf("Using %u threads\n", std::thread::hardware_concurrency());
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    OctreePointAgent agent;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();


    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, &agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, &agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_EQ(o->getItemsCount(), o2->getItemsCount());
    ASSERT_EQ(o->forceGetItemsCount(), o2->forceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    delete []p;
}

TEST_F (OctreeTests, PerformanceSparseInsertAdjustTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    auto oAdjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0);

    OctreePointAgent agent;
    OctreePointAgentAdjust agentAdjust;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, &agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, &agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    start2 = std::chrono::steady_clock::now();
    o2Adjust->insert(p, points, &agentAdjust, true);
    end2 = std::chrono::steady_clock::now();
    diff2 = end2 - start2;
    std::cout << "1 thread with auto adjust: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    oAdjust->insert(p, points, &agentAdjust, true);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;


    ASSERT_EQ(o->getItemsCount(), o2->getItemsCount());
    ASSERT_EQ(o->forceGetItemsCount(), o2->forceGetItemsCount());
    ASSERT_EQ(oAdjust->forceGetItemsCount(), o2Adjust->forceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(*oAdjust == *o2Adjust);

    delete oAdjust;
    delete o2Adjust;

    delete []p;
}

TEST_F (OctreeTests, PerformanceDenseInsertTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    OctreePointAgent agent;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();


    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, &agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, &agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_EQ(o->getItemsCount(), o2->getItemsCount());
    ASSERT_EQ(o->forceGetItemsCount(), o2->forceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    delete []p;
}

TEST_F (OctreeTests, PerformanceDenseInsertAdjustTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8,OctreeVec3<double>(0), 100);

    auto oAdjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0);

    OctreePointAgent agent;
    OctreePointAgentAdjust agentAdjust;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    auto start2 = std::chrono::steady_clock::now();
    o2->insert(p, points, &agent);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;


    auto start = std::chrono::steady_clock::now();
    o->insert(p, points, &agent);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    start2 = std::chrono::steady_clock::now();
    o2Adjust->insert(p, points, &agentAdjust, true);
    end2 = std::chrono::steady_clock::now();
    diff2 = end2 - start2;
    std::cout << "1 thread with auto adjust: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    oAdjust->insert(p, points, &agentAdjust, true);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_EQ(o->getItemsCount(), o2->getItemsCount());
    ASSERT_EQ(o->forceGetItemsCount(), o2->forceGetItemsCount());
    ASSERT_EQ(oAdjust->forceGetItemsCount(), o2Adjust->forceGetItemsCount());
    ASSERT_TRUE(*o == *o2);
    ASSERT_TRUE(*oAdjust == *o2Adjust);

    delete oAdjust;
    delete o2Adjust;

    delete []p;
}

TEST_F (OctreeTests, PerformanceSparseVisitTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    OctreePointAgent agent;
    OctreePointVisitor visitor;
    OctreePointVisitorThreaded visitorThreaded;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, &agent);
    o->insert(p, points, &agent);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(&visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    auto start = std::chrono::steady_clock::now();
    o->visit(&visitorThreaded);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    OctreePointPrinter printer;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    delete []p;
}

TEST_F (OctreeTests, PerformanceSparseVisitAdjustTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    auto oAdjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0);

    OctreePointAgent agent;
    OctreePointAgentAdjust agentAdjust;
    OctreePointVisitor visitor;
    OctreePointVisitorThreaded visitorThreaded;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("test.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, &agent);
    o->insert(p, points, &agent);

    o2Adjust->insert(p, points, &agentAdjust, true);
    oAdjust->insert(p, points, &agentAdjust, true);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(&visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(*oAdjust==*o2Adjust);

    auto start = std::chrono::steady_clock::now();
    o->visit(&visitorThreaded);
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
    o2Adjust->visit(&visitor);
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
    oAdjust->visit(&visitorThreaded);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);

    delete oAdjust;
    delete o2Adjust;

    delete []p;
}

TEST_F (OctreeTests, PerformanceDenseVisitTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    OctreePointAgent agent;
    OctreePointVisitor visitor;
    OctreePointVisitorThreaded visitorThreaded;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, &agent);
    o->insert(p, points, &agent);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(&visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    auto start = std::chrono::steady_clock::now();
    o->visit(&visitorThreaded);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "All threads: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);
    delete []p;
}

TEST_F (OctreeTests, PerformanceDenseVisitAdjustTests) {
    o = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100, 0);
    o2 = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 100);

    auto oAdjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0, 0);
    auto o2Adjust = new Octree<Point, Point, double>(8, OctreeVec3<double>(0), 0);

    OctreePointAgent agent;
    OctreePointAgentAdjust agentAdjust;
    OctreePointVisitor visitor;
    OctreePointVisitorThreaded visitorThreaded;
    Point *p = new Point[points];
    std::fstream outputFile;

    outputFile.open("denseTest.txt", std::ios::in | std::ios::binary);
    outputFile.read((char *) p, points * sizeof(Point));
    outputFile.close();

    o2->insert(p, points, &agent);
    o->insert(p, points, &agent);

    o2Adjust->insert(p, points, &agentAdjust, true);
    oAdjust->insert(p, points, &agentAdjust, true);

    auto start2 = std::chrono::steady_clock::now();
    o2->visit(&visitor);
    auto end2 = std::chrono::steady_clock::now();
    auto diff2 = end2 - start2;
    std::cout << "1 thread: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    Point point = testPoint;
    testPoint.mass = 0.0f;
    testPoint.position = glm::vec3(0);

    ASSERT_TRUE(*o==*o2);
    ASSERT_TRUE(*oAdjust==*o2Adjust);

    auto start = std::chrono::steady_clock::now();
    o->visit(&visitorThreaded);
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
    o2Adjust->visit(&visitor);
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
    o->visit(&visitorThreaded);
    end = std::chrono::steady_clock::now();
    diff = end - start;
    std::cout << "All threads with auto adjust: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    ASSERT_FLOAT_EQ(point.mass, testPoint.mass);
    ASSERT_FLOAT_EQ(point.position.x, testPoint.position.x);
    ASSERT_FLOAT_EQ(point.position.y, testPoint.position.y);
    ASSERT_FLOAT_EQ(point.position.z, testPoint.position.z);

    delete oAdjust;
    delete o2Adjust;

    delete []p;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
