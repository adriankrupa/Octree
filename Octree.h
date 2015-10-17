///////////////////////////////////////////////////////////////////////////////////
//  Octree c++ implementation
//
//  Copyright (c) [2015] [Adrian Krupa]
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////

#ifndef __AKOctree__Octree__
#define __AKOctree__Octree__

#include <array>
#include <thread>
#include <numeric>
#include <atomic>
#include <mutex>

namespace AKOctree {

    template<class LeafDataType, class NodeDataType, class Precision>
    class Octree;

    template<class LeafDataType, class NodeDataType, class Precision>
    class OctreeCell;

    template<class Precision = float>
    struct OctreeVec3 {

        static_assert(std::is_arithmetic<Precision>::value, "Precision must be arithmetic!");

        Precision x = Precision(0);
        Precision y = Precision(0);
        Precision z = Precision(0);

        OctreeVec3() : x(0), y(0), z(0) {}
        explicit OctreeVec3(Precision x) : x(x), y(x), z(x) {}
        OctreeVec3(Precision x, Precision y, Precision z) : x(x), y(y), z(z) {}

        OctreeVec3& operator+=(const OctreeVec3& rhs);
        OctreeVec3& operator-=(const OctreeVec3& rhs);
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class OctreeAgent {
    public:
        virtual ~OctreeAgent() {}
        virtual bool isItemOverlappingCell(const LeafDataType *item,
                                           const OctreeVec3<Precision> &cellCenter,
                                           const Precision &cellRadius) const = 0;
    protected:
        OctreeAgent() {}
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class OctreeAgentAutoAdjustExtension {
    public:
        virtual ~OctreeAgentAutoAdjustExtension() {}
        virtual OctreeVec3<Precision> GetMaxValuesForAutoAdjust(const LeafDataType *item,
                                                                const OctreeVec3<Precision> &max) const = 0;
        virtual OctreeVec3<Precision> GetMinValuesForAutoAdjust(const LeafDataType *item,
                                                                const OctreeVec3<Precision> &min) const = 0;
    protected:
        OctreeAgentAutoAdjustExtension() {}
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class OctreeNodeDataPrinter {
    public:
        virtual std::string GetDataString(NodeDataType& nodeData) const = 0;
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class OctreeVisitor {
    public:
        virtual ~OctreeVisitor() {}

        virtual void visitRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const;
        virtual void visitBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * const cell,
                                 const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const;
        virtual void visitLeaf(const OctreeCell<LeafDataType, NodeDataType, Precision> * const cell,
                               const std::vector<const LeafDataType *> &items) const;

    protected:
        OctreeVisitor() {}
        void ContinueVisit(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > cell) const;
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class OctreeVisitorThreaded {

        template<class L, class N, class P>
        friend class Octree;

        template<class L, class N, class P>
        friend class OctreeCell;

    public:
        virtual ~OctreeVisitorThreaded() {}
        virtual void visitPreRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const {}

        virtual void visitPreBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                    const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8],
                                    std::array<bool, 8>& childsToProcess) const {}

        virtual void visitPostRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const {}

        virtual void visitPostBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                     const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const {}

        virtual void visitLeaf(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                               const std::vector<const LeafDataType *> &items) const {}

    protected:
        OctreeVisitorThreaded() {}

    private:
        void visitRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const;

        void visitBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                         const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const;
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class OctreeCell {

        template<class L, class N, class P>
        friend class Octree;

        template<class L, class N, class P>
        friend class OctreeVisitor;

        template<class L, class N, class P>
        friend class OctreeVisitorThreaded;

        enum class OctreeCellType {
            Leaf,
            Branch
        };

    public:
        OctreeCell(unsigned int          maxItemsPerCell,
                   OctreeVec3<Precision> center,
                   Precision             radius,
                   unsigned int          cellIndex = 0,
                   OctreeCellType        cellType = OctreeCellType::Leaf) : maxItemsPerCell(maxItemsPerCell),
                                                                            center(center),
                                                                            radius(radius),
                                                                            cellType(cellType),
                                                                            cellIndex(cellIndex),
                                                                            internalCellType(cellType) {}

        NodeDataType& getNodeData() const;
        unsigned int getCellIndex() const { return cellIndex; }
        Precision getRadius() const { return radius; }

    private:

        bool getItemPath(const LeafDataType *item, std::string &path) const;
        std::string getStringRepresentation(unsigned int level) const;
        void printTreeAndSubtreeData(unsigned int level, OctreeNodeDataPrinter<LeafDataType, NodeDataType, Precision> *printer) const;
        bool insert(const LeafDataType *item, const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent);
        bool insertInThread(const LeafDataType *item, const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent);
        bool insertIntoLeaf(const LeafDataType *item, const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent);
        void moveCell(OctreeVec3<Precision> center, Precision radius);
        unsigned int forceCountItems() const;
        const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > * getChilds() const { return childs; }
        const std::vector<const LeafDataType *>& getData() const { return data; }
        void visit(const OctreeVisitor<LeafDataType, NodeDataType, Precision> *visitor) const;
        void visit(const OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision>*visitor) const;
        bool isLeaf() const { return cellType == OctreeCellType::Leaf; }
        std::mutex& getMutex() { return nodeMutex; }
        bool isEqual(OctreeCell<LeafDataType, NodeDataType, Precision>  const &rhs) const;
        void makeBranch(const std::vector<const LeafDataType *> &items, const LeafDataType *item, const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent);

        friend bool operator==(const OctreeCell<LeafDataType, NodeDataType, Precision>  &lhs, const OctreeCell<LeafDataType, NodeDataType, Precision>  &rhs) { return lhs.isEqual(rhs); }
        friend bool operator!=(OctreeCell<LeafDataType, NodeDataType, Precision>  const &lhs, OctreeCell<LeafDataType, NodeDataType, Precision>  const &rhs) { return !(lhs == rhs); }

        const unsigned int maxItemsPerCell;
        OctreeVec3<Precision> center = OctreeVec3<Precision>();
        Precision radius = Precision(0);
        OctreeCellType cellType;
        const unsigned int cellIndex;
        OctreeCellType internalCellType;
        mutable NodeDataType nodeData = {};
        std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8];
        std::vector<const LeafDataType *> data;
        std::mutex nodeMutex;
    };

    template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
    class Octree {

        static_assert( std::is_arithmetic<Precision>::value, "Precision must be arithmetic!");

    public:
        Octree(unsigned int maxItemsPerCell,
               unsigned int threadsNumber = 1) : Octree(maxItemsPerCell,
                                                        OctreeVec3<Precision>(),
                                                        Precision(10),
                                                        threadsNumber) {}

        Octree(unsigned int             maxItemsPerCell,
               OctreeVec3<Precision>    center,
               Precision                radius,
               unsigned int             threadsNumber = 1);

        unsigned int getMaxItemsPerCell() const {  return maxItemsPerCell; }
        unsigned int getItemsCount() const { return itemsCount; }
        void clear();
        void insert(const LeafDataType *item, const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent);
        void insert(const LeafDataType *items,
                    const unsigned int itemsCount,
                    const OctreeAgent<LeafDataType, NodeDataType, Precision> *agentInsert,
                    const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision> *agentAdjust,
                    bool autoAdjustTree = true);

        template <class T>
        void insert(const LeafDataType *items, const unsigned int itemsCount, const T *agent, bool autoAdjustTree);

        template <class T>
        void insert(const LeafDataType *items, const unsigned int itemsCount, const T *agent);
        void insert(std::vector<LeafDataType> &items,
                    const OctreeAgent<LeafDataType, NodeDataType, Precision> *agentInsert,
                    const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision> *agentAdjust,
                    bool autoAdjustTree = true);

        template <class T>
        void insert(std::vector<LeafDataType> &items, const T *agent, bool autoAdjustTree);

        template <class T>
        void insert(std::vector<LeafDataType> &items, const T *agent);
        std::string getItemPath(LeafDataType *item) const;
        std::string getStringRepresentation() const { return root->getStringRepresentation(0); }
        void printTreeData(OctreeNodeDataPrinter<LeafDataType, NodeDataType, Precision> *printer) const {  root->printTreeAndSubtreeData(0, printer); }
        unsigned int forceGetItemsCount() const { return root->forceCountItems();  }
        void visit(const OctreeVisitor<LeafDataType, NodeDataType, Precision> *visitor) const;
        void visit(const OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision> *visitor) const;
        bool operator==(const Octree<LeafDataType, NodeDataType, Precision> &rhs) { return *root == *rhs.root; }
        bool operator!=(const Octree<LeafDataType, NodeDataType, Precision> &rhs) { return *root != *rhs.root; }

    private:
        void insertThread(const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent);

        void visitThread(const OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision> *visitor,
                         std::array<std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> >, 8 > threadRoots,
                         std::vector<int> toVisit) const;


        OctreeVec3<Precision> center = OctreeVec3<Precision>();
        Precision radius = Precision(10);

        const unsigned int maxItemsPerCell = 1;
        std::atomic_uint itemsCount;

        unsigned int threadsNumber = 1;
        std::mutex threadsMutex;

        std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > root;
        std::vector<const LeafDataType *> itemsToAdd;
        mutable std::vector<std::thread> threads;
    };

    template<class P> // P=Precision
    OctreeVec3<P>& OctreeVec3<P>::operator+=(const OctreeVec3<P>& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    template<class P> // P=Precision
    OctreeVec3<P>& OctreeVec3<P>::operator-=(const OctreeVec3<P>& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    template<class P> // P=Precision
    OctreeVec3<P> operator+(OctreeVec3<P> lhs, const OctreeVec3<P>& rhs) {
        lhs += rhs;
        return lhs;
    }

    template<class P> // P=Precision
    OctreeVec3<P> operator/(OctreeVec3<P> lhs, const P& rhs) {
        lhs.x /= rhs;
        lhs.y /= rhs;
        lhs.z /= rhs;
        return lhs;
    }

    template<class P> // P=Precision
    OctreeVec3<P> operator-(OctreeVec3<P> lhs, const OctreeVec3<P>& rhs) {
        lhs -= rhs;
        return lhs;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeVisitor<L, N, P>::visitRoot(const std::shared_ptr<OctreeCell<L, N, P> > rootCell) const {
        ContinueVisit(rootCell);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeVisitor<L, N, P>::visitBranch(const OctreeCell<L, N, P> * cell,
                                             const std::shared_ptr<OctreeCell<L, N, P> > childs[8]) const {

        for (int i = 0; i < 8; ++i) {
            ContinueVisit(childs[i]);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeVisitor<L, N, P>::visitLeaf(const OctreeCell<L, N, P> * cell,
                                           const std::vector<const L *> &items) const {}

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeVisitor<L, N, P>::ContinueVisit(const std::shared_ptr<OctreeCell<L, N, P> > cell) const {
        cell->visit(this);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeVisitorThreaded<L, N, P>::visitRoot(const std::shared_ptr<OctreeCell<L, N, P> > rootCell) const {
        visitPreRoot(rootCell);
        rootCell->visit(this);
        visitPostRoot(rootCell);
    }

    std::array<bool, 8> getArrayOfChildsToProcess() {
        std::array<bool, 8> a;
        a.fill(true);
        return a;
    }

    template<class LeafDataType, class NodeDataType, class Precision>
    void OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision>::visitBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                                                                   const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const {
        auto childsToProcess = getArrayOfChildsToProcess();
        visitPreBranch(cell, childs, childsToProcess);
        for (int i = 0; i < 8; ++i) {
            if(childsToProcess[i]) {
                childs[i]->visit(this);
            }
        }
        visitPostBranch(cell, childs);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    N& OctreeCell<L, N, P>::getNodeData() const {
        return nodeData;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    bool OctreeCell<L, N, P>::getItemPath(const L *item, std::string &path) const {
        if(internalCellType == OctreeCellType::Leaf) {
            for (auto &it : data) {
                if (it == item) {
                    return true;
                }
            }
            return false;
        } else {
            for (int i = 0; i < 8; ++i) {
                path += std::to_string(i);
                if (childs[i]->getItemPath(item, path)) {
                    return true;
                } else {
                    path.pop_back();
                }
            }
            return false;
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    std::string OctreeCell<L, N, P>::getStringRepresentation(unsigned int level) const {
        std::string s;
        if(internalCellType == OctreeCellType::Leaf) {
            s += "Leaf, items:" + std::to_string(data.size());
            for (unsigned int i = 0; i < data.size(); ++i) {
                s += " " + std::to_string((unsigned long long)data[i]);
            }
            s += "\n";
        } else {
            s += "Branch ";
            for (int i = 0; i < 8; ++i) {
                if (i != 0) {
                    for (unsigned int j = 0; j < level + 1; ++j) {
                        s += "       ";
                    }
                    for (unsigned int j = 0; j < level; ++j) {
                        s += "  ";
                    }
                }
                s += std::to_string(i) + " " + childs[i]->getStringRepresentation(level + 1);
            }
        }
        return s;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeCell<L, N, P>::printTreeAndSubtreeData(unsigned int level, OctreeNodeDataPrinter<L, N, P> *printer) const {
        if(internalCellType == OctreeCellType::Leaf) {
            printf("Leaf: %s\n", printer->GetDataString(nodeData).c_str());
        } else {
            printf("Branch ");
            printf("%s", printer->GetDataString(nodeData).c_str());
            for (int i = 0; i < 8; ++i) {
                if (i != 0) {
                    for (unsigned int j = 0; j < level + 1; ++j) {
                        printf("       ");
                    }
                    for (unsigned int j = 0; j < level; ++j) {
                        printf("  ");
                    }
                }
                printf("%d ", i);
                if (childs[i] == nullptr) {
                    printf("NULL\n");
                } else {
                    childs[i]->printTreeAndSubtreeData(level + 1, printer);
                }
            }
        }
    }

    template<class P>
    OctreeVec3<P> getCenterDelta(int index, P halfRadius) {
        /*
           y
           |
           +--x
          /
         z
             0----1
            /|   /|
           2-+--3 |
           | 4--+-5
           |/   |/
           6----7

           000---001
           /|    /|
          / |   / |
        010-+-011 |
         | 100-+-101
         | /   | /
         |/    |/
        110---111
         */


        bool up = index < 4;//+y
        bool right = (bool) (index & 1);//+x
        bool front = (bool) (index & 2); //+z
        return OctreeVec3<P>(right ? halfRadius : -halfRadius,
                             up ? halfRadius : -halfRadius,
                             front ? halfRadius : -halfRadius);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    bool OctreeCell<L, N, P>::insert(const L *item, const OctreeAgent<L, N, P> *agent) {
        if(internalCellType == OctreeCellType::Leaf) {
            return insertIntoLeaf(item, agent);
        } else {
            P halfRadius = this->radius / P(2);
            for (int i = 0; i < 8; ++i) {
                OctreeVec3<P> newCenter = this->center + getCenterDelta(i, halfRadius);
                if (agent->isItemOverlappingCell(item, newCenter, halfRadius)) {
                    return childs[i]->insert(item, agent);
                }
            }
            return false;
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    bool OctreeCell<L, N, P>::insertInThread(const L *item, const OctreeAgent<L, N, P> *agent) {
        if(internalCellType == OctreeCellType::Leaf) {
            return insertIntoLeaf(item, agent);
        } else {
            P halfRadius = this->radius / P(2);
            for (int i = 0; i < 8; ++i) {
                OctreeVec3<P> newCenter = this->center + getCenterDelta(i, halfRadius);
                if (agent->isItemOverlappingCell(item, newCenter, halfRadius)) {
                    if(childs[i]->isLeaf()) {
                        std::lock_guard<std::mutex> lock(childs[i]->getMutex());
                        return childs[i]->insertInThread(item, agent);
                    } else {
                        return childs[i]->insertInThread(item, agent);
                    }
                }
            }
            return true;
        }
    }

    // https://en.wikipedia.org/wiki/Substitution_failure_is_not_an_error
    namespace sfinae {
        template<class>
        struct sfinae_true : std::true_type {};

        template<class T, class A0>
        static auto test_equalityOp(int) -> sfinae_true<decltype(*std::declval<T>() == *std::declval<T>())>;

        template<class, class A0>
        static auto test_equalityOp(long) -> std::false_type;

        template<class T, class L, bool>
        struct equality {
            static inline bool isEqual(T t, L l) {return t == l;}
        };

        template<class T, class L>
        struct equality<T, L, true> {
            static inline bool isEqual(T t, L l) {return t == l || *t == *l;}
        };

        template<class T, class Arg>
        struct pointer_equality : decltype(test_equalityOp<T, Arg>(0)) {
            static_assert(std::is_pointer<T>::value && std::is_pointer<Arg>::value, "Template arguments must be pointers");

            static bool isEqual(T t, Arg l) {
                return equality<T, Arg, decltype(test_equalityOp<T, Arg>(0))::value>::isEqual(t, l);
            }
        };
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    bool OctreeCell<L, N, P>::insertIntoLeaf(const L *item, const OctreeAgent<L, N, P> *agent) {
        for(auto& d : data) {
            if (sfinae::pointer_equality<const L*, const L*>::isEqual(item, d)) {
                return false;
            }
        }
        if (maxItemsPerCell <= data.size()) {
            makeBranch(data, item, agent);
        } else {
            data.push_back(item);
        }
        return true;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeCell<L, N, P>::moveCell(OctreeVec3<P> center, P radius) {
        assert(this->isLeaf());
        this->center = center;
        this->radius = radius;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    unsigned int OctreeCell<L, N, P>::forceCountItems() const {
        if(internalCellType == OctreeCellType::Leaf) {
            return (unsigned int)data.size();
        } else {
            unsigned int items = 0;
            for (int i = 0; i < 8; ++i) {
                items += childs[i]->forceCountItems();
            }
            return items;
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeCell<L, N, P>::visit(const OctreeVisitor<L, N, P> *visitor) const {
        if(internalCellType == OctreeCellType::Leaf) {
            visitor->visitLeaf(this, data);
        } else {
            visitor->visitBranch(this, childs);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeCell<L, N, P>::visit(const OctreeVisitorThreaded<L, N, P> *visitor) const {
        if(internalCellType == OctreeCellType::Leaf) {
            visitor->visitLeaf(this, data);
        } else {
            visitor->visitBranch(this, childs);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    bool OctreeCell<L, N, P>::isEqual(OctreeCell<L, N, P>  const &rhs) const {
        if(internalCellType != rhs.internalCellType) {
            return false;
        }
        if(internalCellType == OctreeCellType::Leaf) {
            if (data.size() != rhs.data.size()) {
                return false;
            }
            for (unsigned int i = 0; i < data.size(); ++i) {
                bool found = false;
                for (unsigned int j = 0; j < data.size(); ++j) {
                    if (data[i] == rhs.data[j]) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    return false;
                }
            }
        } else {
            for (int i = 0; i < 8; ++i) {
                bool equal = *childs[i] == *rhs.childs[i];
                if (!equal) {
                    return false;
                }
            }
        }
        return true;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void OctreeCell<L, N, P>::makeBranch(const std::vector<const L *> &items, const L *item, const OctreeAgent<L, N, P> *agent) {
        float halfRadius = radius / 2.0f;
        for (int i = 0; i < 8; ++i) {

            bool up = i < 4;//+y
            bool right = (i & 1) == 1;//+x
            bool front = ((i >> 1) & 1) != 0; //+z
            OctreeVec3<P> newCenter = this->center + OctreeVec3<P>(right ? halfRadius : -halfRadius,
                                                                   up ? halfRadius : -halfRadius,
                                                                   front ? halfRadius : -halfRadius);
            childs[i] = std::make_shared<OctreeCell<L, N, P> >(maxItemsPerCell, newCenter, halfRadius, i);
        }
        internalCellType = OctreeCellType::Branch;
        for (unsigned int i = 0; i < items.size(); ++i) {
            this->insert(items[i], agent);
        }
        this->insert(item, agent);
        cellType = OctreeCellType::Branch;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    Octree<L, N, P>::Octree(unsigned int maxItemsPerCell,
                            OctreeVec3<P> center,
                            P radius,
                            unsigned int threadsNumber) : center(center),
                                                          radius(radius),
                                                          maxItemsPerCell(maxItemsPerCell),
                                                          threadsNumber(threadsNumber) {
        if(threadsNumber == 0) {
            this->threadsNumber = std::thread::hardware_concurrency();
        }
        root = std::make_shared<OctreeCell<L, N, P> >(maxItemsPerCell, center, radius);
        itemsCount.store(0);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::clear() {
        root = std::make_shared<OctreeCell<L, N, P> >(maxItemsPerCell, center, radius);
        itemsCount.store(0);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::insert(const L *item, const OctreeAgent<L, N, P> *agent) {
        if (agent->isItemOverlappingCell(item, center, radius)) {
            if(root->insert(item, agent)) {
                itemsCount.fetch_add(1);
            }
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::insert(const L *items,
                                 const unsigned int itemsCount,
                                 const OctreeAgent<L, N, P> *agentInsert,
                                 const OctreeAgentAutoAdjustExtension<L, N, P> *agentAdjust,
                                 bool autoAdjustTree) {

        assert((autoAdjustTree || radius > P(0)) && "Radius has to be > 0");

        if (autoAdjustTree && agentAdjust != nullptr && this->itemsCount == 0) {

            OctreeVec3<P> max = center + OctreeVec3<P>(radius);
            OctreeVec3<P> min = center - OctreeVec3<P>(radius);

            for (unsigned int i = 0; i < itemsCount; ++i) {
                max = agentAdjust->GetMaxValuesForAutoAdjust(&items[i], max);
                min = agentAdjust->GetMinValuesForAutoAdjust(&items[i], min);
            }
            center = (max + min) / P(2);
            radius = std::max(std::abs(center.x - max.x), std::abs(center.y - max.y));
            radius = std::max(radius, glm::abs(center.z - max.z));
            root->moveCell(center, radius);
        }

        if (threadsNumber != 1) {
            itemsToAdd.reserve(itemsCount);
            for (unsigned int i = 0; i < itemsCount; ++i) {
                itemsToAdd.push_back(&items[i]);
            }
            for (unsigned int i = 0; i < threadsNumber; ++i) {
                threads.push_back(std::thread(&Octree::insertThread, this, agentInsert));
            }
            for (unsigned int i = 0; i < threadsNumber; ++i) {
                threads[i].join();
            }
            threads.clear();
        } else {
            for (unsigned int i = 0; i < itemsCount; ++i) {
                if (agentInsert->isItemOverlappingCell(&items[i], center, radius)) {
                    if(root->insert(&items[i], agentInsert)) {
                        this->itemsCount++;
                    }
                }
            }
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    template <class T>
    void Octree<L, N, P>::insert(const L *items,
                                 const unsigned int itemsCount,
                                 const T *agent,
                                 bool autoAdjustTree) {

        static_assert(std::is_base_of<OctreeAgent<L, N, P>, T>::value, "Agent has wrong class");

        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<L, N, P>* > (agent);
        if(autoAdjustTree && adj == nullptr) {
            //printf("WARNING: To use auto adjust agent has to implement OctreeAgentAutoAdjustExtension\n");
            insert(items, itemsCount, agent, nullptr, false);
        } else {
            insert(items, itemsCount, agent, adj, autoAdjustTree);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    template <class T>
    void Octree<L, N, P>::insert(const L *items,
                                 const unsigned int itemsCount,
                                 const T *agent) {

        static_assert(std::is_base_of<OctreeAgent<L, N, P>, T>::value, "Agent has wrong class");
        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<L, N, P>* > (agent);
        insert(items, itemsCount, agent, adj, adj != nullptr);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::insert(std::vector<L> &items,
                                 const OctreeAgent<L, N, P> *agentInsert,
                                 const OctreeAgentAutoAdjustExtension<L, N, P> *agentAdjust,
                                 bool autoAdjustTree) {

        if (autoAdjustTree && agentAdjust != nullptr && this->itemsCount == 0) {

            OctreeVec3<P> max = center + OctreeVec3<P>(radius);
            OctreeVec3<P> min = center - OctreeVec3<P>(radius);

            for (auto& item : items) {
                max = agentAdjust->GetMaxValuesForAutoAdjust(&item, max);
                min = agentAdjust->GetMinValuesForAutoAdjust(&item, min);
            }
            center = (max + min) / P(2);
            radius = std::max(std::abs(center.x - max.x), std::abs(center.y - max.y));
            radius = std::max(radius, glm::abs(center.z - max.z));
            root->moveCell(center, radius);

        }

        if (threadsNumber != 1) {
            itemsToAdd.reserve(items.size());
            for (unsigned int i = 0; i < items.size(); ++i) {
                itemsToAdd.push_back(&items[i]);
            }
            if (threadsNumber == 0) {
                threadsNumber = std::thread::hardware_concurrency();
            }
            threads.clear();
            for (unsigned int i = 0; i < threadsNumber; ++i) {
                threads.push_back(std::thread(&Octree::insertThread, this, agentInsert));
            }
            for (unsigned int i = 0; i < threadsNumber; ++i) {
                threads[i].join();
            }
            threads.clear();
        } else {
            for (unsigned int i = 0; i < items.size(); ++i) {
                if (agentInsert->isItemOverlappingCell(&items[i], center, radius)) {
                    if(root->insert(&items[i], agentInsert)) {
                        this->itemsCount.fetch_add(1);
                    }
                }
            }
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    template <class T>
    void Octree<L, N, P>::insert(std::vector<L> &items, const T *agent, bool autoAdjustTree) {

        static_assert(std::is_base_of<OctreeAgent<L, N, P>, T>::value, "Agent has wrong class");

        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<L, N, P>* > (agent);
        if(autoAdjustTree && adj == nullptr) {
            //printf("WARNING: To use auto adjust agent has to implement OctreeAgentAutoAdjustExtension\n");
            insert(items, agent, nullptr, false);
        } else {
            insert(items, agent, adj, autoAdjustTree);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    template <class T>
    void Octree<L, N, P>::insert(std::vector<L> &items, const T *agent) {

        static_assert(std::is_base_of<OctreeAgent<L, N, P>, T>::value, "Agent has wrong class");
        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<L, N, P>* > (agent);
        insert(items, agent, adj, adj != nullptr);
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    std::string Octree<L, N, P>::getItemPath(L *item) const {
        std::string v;
        root->getItemPath(item, v);
        return v;
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::visit(const OctreeVisitor<L, N, P> *visitor) const {
        if (threadsNumber != 1) {
            //printf("WARNING: visiting with multiple tree requires OctreeVisitorThreaded\nUsing 1 thread\n");
        }
        visitor->visitRoot(root);
    }

    void pushRangeIntoVector(std::vector<int>& v, int from, int to) {
        for (int i = from; i < to; ++i) {
            v.push_back(i);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::visit(const OctreeVisitorThreaded<L, N, P> *visitor) const {
        if (threadsNumber != 1) {
            visitor->visitPreRoot(root);
            if (root->isLeaf()) {
                root->visit(visitor);
            } else {
                auto childsToVisit = getArrayOfChildsToProcess();
                visitor->visitPreBranch(root.get(), root->getChilds(), childsToVisit);
                int childsToVisitCount = std::count_if(childsToVisit.begin(), childsToVisit.end(), [](bool b) {return b;});
                std::array< std::vector<int>, 16 > toVisit;
                std::array<std::shared_ptr<OctreeCell<L, N, P> >, 8 > roots;
                if(threadsNumber <= 8) {
                    roots[0] = root;
                    for (unsigned int i = 0; i < threadsNumber; ++i) {
                        int from = (childsToVisitCount * i) / threadsNumber;
                        int to = std::min<int>(childsToVisitCount, (childsToVisitCount * (i+1)) / threadsNumber);
                        int skipped = 0;
                        for (int j = from; j < to; ++j) {
                            if(childsToVisit[j+skipped]) {
                                toVisit[i].push_back(j+skipped);
                            } else {
                                j--;
                                skipped++;
                            }
                        }
                        threads.push_back(std::thread(&Octree::visitThread, this, std::ref(visitor), std::ref(roots), std::ref(toVisit[i])));
                    }
                    for (unsigned int i = 0; i < threadsNumber; ++i) {
                        threads[i].join();
                    }
                    threads.clear();

                } else {
                    auto childs = root->getChilds();
                    std::array<std::array<bool, 8>, 8 > childsToProcessArray;

                    for (int i = 0; i < 8; ++i) {
                        childsToProcessArray[i] = getArrayOfChildsToProcess();
                        roots[i] = childs[i];
                    }
                    for (int i = 0; i < 8; ++i) {
                        if(!childsToVisit[i]) {
                            for (int j = 0; j < 8; ++j) {
                                childsToProcessArray[i][j]=false;
                            }
                            continue;
                        }
                        if(childs[i]->isLeaf()) {
                            visitor->visitLeaf(childs[i].get(), childs[i]->getData());
                        } else {
                            visitor->visitPreBranch(childs[i].get(), childs[i]->getChilds(), childsToProcessArray[i]);
                        }
                    }

                    int numberOfElementsToVisit = std::accumulate(childsToProcessArray.begin(), childsToProcessArray.end(), 0,
                                                                  [](const int& counter, std::array<bool, 8> elements) {
                                                                      return counter + std::count_if(elements.begin(),
                                                                                                     elements.end(),
                                                                                                     [](bool b) {return b;});
                                                                  });

                    int accumulator = 0;
                    for (unsigned int i = 0; i < threadsNumber; ++i) {
                        int from = (numberOfElementsToVisit * i) / threadsNumber;
                        int to = std::min<int>(numberOfElementsToVisit, (numberOfElementsToVisit * (i+1)) / threadsNumber);
                        int elementsToPush = to-from;

                        for (int j = 0; j < elementsToPush; ++j) {
                            int rootCellIndex = accumulator/8;
                            int childCellIndex = accumulator%8;
                            if(childsToProcessArray[rootCellIndex][childCellIndex]) {
                                toVisit[i].push_back(accumulator);
                            } else {
                                j--;
                            }
                            accumulator++;
                        }
                        threads.push_back(std::thread(&Octree::visitThread, this, std::ref(visitor), std::ref(roots), std::ref(toVisit[i])));
                    }

                    for (auto& thread : threads) {
                        thread.join();
                    }
                    threads.clear();

                    for (int i = 0; i < 8; ++i) {
                        if(!childs[i]->isLeaf()) {
                            visitor->visitPostBranch(childs[i].get(), childs[i]->getChilds());
                        }
                    }

                }
                visitor->visitPostBranch(root.get(), root->getChilds());
            }

            visitor->visitPostRoot(root);

        } else {
            visitor->visitRoot(root);
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::insertThread(const OctreeAgent<L, N, P> *agent) {
        unsigned int itemsToBatch = threadsNumber * threadsNumber;

        while (!itemsToAdd.empty()) {
            threadsMutex.lock();
            if (itemsToAdd.empty()) {
                threadsMutex.unlock();
                break;
            }
            std::vector<const L *> tempItems;
            tempItems.reserve(itemsToBatch);
            for (unsigned int i = 0; i < itemsToBatch; ++i) {
                if (itemsToAdd.empty()) {
                    break;
                }
                auto item = itemsToAdd[itemsToAdd.size() - 1];
                tempItems.push_back(item);
                itemsToAdd.pop_back();
                this->itemsCount.fetch_add(1);
            }
            threadsMutex.unlock();
            for(auto& item : tempItems) {
                if (agent->isItemOverlappingCell(item, center, radius)) {
                    if(root -> isLeaf()) {
                        root->getMutex().lock();
                        root->insertInThread(item, agent);
                        root->getMutex().unlock();
                    } else {
                        root->insertInThread(item, agent);
                    }
                }
            }
        }
    }

    template<class L, class N, class P> //L=LeafDataType N=NodeDataType P=Precision
    void Octree<L, N, P>::visitThread(const OctreeVisitorThreaded<L, N, P> *visitor,
                                      std::array<std::shared_ptr<OctreeCell<L, N, P> >, 8 > threadRoots,
                                      std::vector<int> toVisit) const {

        for(auto& index : toVisit) {
            int rootIndex = index / 8;
            int cellIndex = index % 8;

            if (!threadRoots[rootIndex]->isLeaf()) {
                auto childs = threadRoots[rootIndex]->getChilds();
                childs[cellIndex] -> visit(visitor);
            } else {
                visitor->visitLeaf(threadRoots[rootIndex].get(), threadRoots[rootIndex]->getData());
            }
        }
    }
}

#endif /* defined(__AKOctree__Octree__) */
