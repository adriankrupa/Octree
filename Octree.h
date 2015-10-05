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

#include <vector>
#include <thread>
#include <mutex>

namespace AKOctree {

#define OctreeTemplateDefault template<class LeafDataType, class NodeDataType = LeafDataType, class Precision = float>
#define OctreeTemplate template<class LeafDataType, class NodeDataType, class Precision>

    OctreeTemplate
    class Octree;

    OctreeTemplate
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

    OctreeTemplateDefault
    class OctreeAgent {
    public:
        virtual ~OctreeAgent() {}
        virtual bool isItemOverlappingCell(const LeafDataType *item,
                                           const OctreeVec3<Precision> &cellCenter,
                                           const Precision &cellRadius) const = 0;
    protected:
        OctreeAgent() {}
    };

    OctreeTemplateDefault
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

    OctreeTemplateDefault
    class OctreeNodeDataPrinter {
    public:
        virtual std::string GetDataString(NodeDataType& nodeData) const = 0;
    };

    OctreeTemplateDefault
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

    OctreeTemplateDefault
    class OctreeVisitorThreaded {

        template<class L, class N, class P>
        friend class Octree;

        template<class L, class N, class P>
        friend class OctreeCell;

    public:
        virtual ~OctreeVisitorThreaded() {}
        virtual void visitPreRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const {}

        virtual void visitPreBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                    const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const {}

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

    enum class OctreeCellType {
        Leaf,
        Branch
    };

    OctreeTemplateDefault
    class OctreeCell {

        typedef OctreeCell<LeafDataType, NodeDataType, Precision> OctreeCellLNP;
        typedef OctreeAgent<LeafDataType, NodeDataType, Precision> OctreeAgentLNP;
        typedef OctreeVisitor<LeafDataType, NodeDataType, Precision> OctreeVisitorLNP;
        typedef OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision> OctreeVisitorThreadedLNP;
        typedef OctreeVec3<Precision> OctreeVec3p;

        template<class L, class N, class P>
        friend class Octree;

        template<class L, class N, class P>
        friend class OctreeVisitor;

        template<class L, class N, class P>
        friend class OctreeVisitorThreaded;

    public:
        OctreeCell(unsigned int     maxItemsPerCell,
                   OctreeVec3p      center,
                   Precision        radius,
                   unsigned int     cellIndex = 0,
                   OctreeCellType   cellType = OctreeCellType::Leaf) : maxItemsPerCell(maxItemsPerCell),
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
        bool insert(const LeafDataType *item, const OctreeAgentLNP *agent);
        void moveCell(OctreeVec3<Precision> center, Precision radius);
        unsigned int forceCountItems() const;
        const std::shared_ptr<OctreeCellLNP> * getChilds() const { return childs; }
        const std::vector<const LeafDataType *>& getData() const { return data; }
        void visit(const OctreeVisitorLNP *visitor) const;
        void visit(const OctreeVisitorThreadedLNP *visitor) const;
        bool isLeaf() const { return cellType == OctreeCellType::Leaf; }
        std::mutex& getMutex() { return nodeMutex; }
        bool insertInThread(const LeafDataType *item, const OctreeAgentLNP *agent);
        bool isEqual(OctreeCellLNP const &rhs) const;
        void makeBranch(const std::vector<const LeafDataType *> &items, const LeafDataType *item, const OctreeAgentLNP *agent);

        friend bool operator==(const OctreeCellLNP &lhs, const OctreeCellLNP &rhs) { return lhs.isEqual(rhs); }
        friend bool operator!=(OctreeCellLNP const &lhs, OctreeCellLNP const &rhs) { return !(lhs == rhs); }

        const unsigned int maxItemsPerCell;
        OctreeVec3p center = OctreeVec3p();
        Precision radius = 0.0f;
        OctreeCellType cellType;
        const unsigned int cellIndex;
        OctreeCellType internalCellType;
        mutable NodeDataType nodeData;
        std::shared_ptr<OctreeCellLNP> childs[8];
        std::vector<const LeafDataType *> data;
        std::mutex nodeMutex;
    };

    OctreeTemplateDefault
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
                         std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > threadRoots[2],
                         int* from,
                         int* to,
                         int size) const;


        OctreeVec3<Precision> center = OctreeVec3<Precision>();
        Precision radius = Precision(10);

        const unsigned int maxItemsPerCell = 1;
        unsigned int itemsCount = 0;

        unsigned int threadsNumber = 1;
        std::mutex threadsMutex;

        std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > root;
        std::vector<const LeafDataType *> itemsToAdd;
        mutable std::vector<std::thread> threads;
    };

    template<class Precision>
    OctreeVec3<Precision>& OctreeVec3<Precision>::operator+=(const OctreeVec3<Precision>& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    template<class Precision>
    OctreeVec3<Precision>& OctreeVec3<Precision>::operator-=(const OctreeVec3<Precision>& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    template<class Precision>
    OctreeVec3<Precision> operator+(OctreeVec3<Precision> lhs, const OctreeVec3<Precision>& rhs) {
        lhs += rhs;
        return lhs;
    }

    template<class Precision>
    OctreeVec3<Precision> operator/(OctreeVec3<Precision> lhs, const Precision& rhs) {
        lhs.x /= rhs;
        lhs.y /= rhs;
        lhs.z /= rhs;
        return lhs;
    }

    template<class Precision>
    OctreeVec3<Precision> operator-(OctreeVec3<Precision> lhs, const OctreeVec3<Precision>& rhs) {
        lhs -= rhs;
        return lhs;
    }

    OctreeTemplate
    void OctreeVisitor<LeafDataType, NodeDataType, Precision>::visitRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const {
        ContinueVisit(rootCell);
    }

    OctreeTemplate
    void OctreeVisitor<LeafDataType, NodeDataType, Precision>::visitBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                                                           const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const {
        for (int i = 0; i < 8; ++i) {
            ContinueVisit(childs[i]);
        }
    }

    OctreeTemplate
    void OctreeVisitor<LeafDataType, NodeDataType, Precision>::visitLeaf(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                                                         const std::vector<const LeafDataType *> &items) const {};

    OctreeTemplate
    void OctreeVisitor<LeafDataType, NodeDataType, Precision>::ContinueVisit(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > cell) const {
        cell->visit(this);
    }

    OctreeTemplate
    void OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision>::visitRoot(const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > rootCell) const {
        visitPreRoot(rootCell);
        rootCell->visit(this);
        visitPostRoot(rootCell);
    }

    OctreeTemplate
    void OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision>::visitBranch(const OctreeCell<LeafDataType, NodeDataType, Precision> * cell,
                                                                                   const std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > childs[8]) const {
        visitPreBranch(cell, childs);
        for (int i = 0; i < 8; ++i) {
            childs[i]->visit(this);
        }
        visitPostBranch(cell, childs);
    }

    OctreeTemplate
    NodeDataType& OctreeCell<LeafDataType, NodeDataType, Precision>::getNodeData() const {
        return nodeData;
    }

    OctreeTemplate
    bool OctreeCell<LeafDataType, NodeDataType, Precision>::getItemPath(const LeafDataType *item, std::string &path) const {
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

    OctreeTemplate
    std::string OctreeCell<LeafDataType, NodeDataType, Precision>::getStringRepresentation(unsigned int level) const {
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

    OctreeTemplate
    void OctreeCell<LeafDataType, NodeDataType, Precision>::printTreeAndSubtreeData(unsigned int level, OctreeNodeDataPrinter<LeafDataType, NodeDataType, Precision> *printer) const {
        if(internalCellType == OctreeCellType::Leaf) {
            printf("Leaf: %s\n", printer->GetDataString(nodeData).c_str());
        } else {
            printf("Branch ");
            printf(printer->GetDataString(nodeData).c_str());
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

    OctreeTemplate
    bool OctreeCell<LeafDataType, NodeDataType, Precision>::insert(const LeafDataType *item, const OctreeAgentLNP *agent) {
        if(internalCellType == OctreeCellType::Leaf) {
            for (unsigned int i = 0; i < data.size(); ++i) {
                if (data[i] == item) {
                    return false;
                }

            }
            if (maxItemsPerCell <= data.size()) {
                makeBranch(data, item, agent);
            } else {
                data.push_back(item);
            }
            return true;
        } else {
            Precision halfRadius = this->radius / Precision(2);
            for (int i = 0; i < 8; ++i) {
                bool up = i < 4;//+y
                bool right = (i & 1) == 1;//+x
                bool front = ((i >> 1) & 1) != 0; //+z
                OctreeVec3p newCenter = this->center + OctreeVec3p(right ? halfRadius : -halfRadius,
                                                                   up ? halfRadius : -halfRadius,
                                                                   front ? halfRadius : -halfRadius);
                if (agent->isItemOverlappingCell(item, newCenter, halfRadius)) {
                    return childs[i]->insert(item, agent);
                }
            }
            return false;
        }
    }

    OctreeTemplate
    void OctreeCell<LeafDataType, NodeDataType, Precision>::moveCell(OctreeVec3<Precision> center, Precision radius) {
        this->center = center;
        this->radius = radius;
    }

    OctreeTemplate
    unsigned int OctreeCell<LeafDataType, NodeDataType, Precision>::forceCountItems() const {
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

    OctreeTemplate
    void OctreeCell<LeafDataType, NodeDataType, Precision>::visit(const OctreeVisitorLNP *visitor) const {
        if(internalCellType == OctreeCellType::Leaf) {
            visitor->visitLeaf(this, data);
        } else {
            visitor->visitBranch(this, childs);
        }
    }

    OctreeTemplate
    void OctreeCell<LeafDataType, NodeDataType, Precision>::visit(const OctreeVisitorThreadedLNP *visitor) const {
        if(internalCellType == OctreeCellType::Leaf) {
            visitor->visitLeaf(this, data);
        } else {
            visitor->visitBranch(this, childs);
        }
    }

    OctreeTemplate
    bool OctreeCell<LeafDataType, NodeDataType, Precision>::insertInThread(const LeafDataType *item, const OctreeAgentLNP *agent) {
        if(internalCellType == OctreeCellType::Leaf) {
            for (unsigned int i = 0; i < data.size(); ++i) {
                if (data[i] == item) {
                    return false;
                }
            }
            if (maxItemsPerCell <= data.size()) {
                makeBranch(data, item, agent);
            } else {
                data.push_back(item);
            }
            return true;
        } else {
            Precision halfRadius = this->radius / Precision(2);
            for (int i = 0; i < 8; ++i) {
                bool up = i < 4;//+y
                bool right = (i & 1) == 1;//+x
                bool front = ((i >> 1) & 1) != 0; //+z
                OctreeVec3p newCenter = this->center + OctreeVec3p(right ? halfRadius : -halfRadius,
                                                                   up ? halfRadius : -halfRadius,
                                                                   front ? halfRadius : -halfRadius);

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

    OctreeTemplate
    bool OctreeCell<LeafDataType, NodeDataType, Precision>::isEqual(OctreeCellLNP const &rhs) const {
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

    OctreeTemplate
    void OctreeCell<LeafDataType, NodeDataType, Precision>::makeBranch(const std::vector<const LeafDataType *> &items, const LeafDataType *item, const OctreeAgentLNP *agent) {
        float halfRadius = radius / 2.0f;
        for (int i = 0; i < 8; ++i) {

            bool up = i < 4;//+y
            bool right = (i & 1) == 1;//+x
            bool front = ((i >> 1) & 1) != 0; //+z
            OctreeVec3<Precision> newCenter = this->center + OctreeVec3<Precision>(right ? halfRadius : -halfRadius,
                                                                                   up ? halfRadius : -halfRadius,
                                                                                   front ? halfRadius : -halfRadius);
            childs[i] = std::make_shared<OctreeCellLNP>(maxItemsPerCell, newCenter, halfRadius, i);
        }
        internalCellType = OctreeCellType::Branch;
        for (unsigned int i = 0; i < items.size(); ++i) {
            this->insert(items[i], agent);
        }
        this->insert(item, agent);
        cellType = OctreeCellType::Branch;
    }

    OctreeTemplate
    Octree<LeafDataType, NodeDataType, Precision>::Octree(unsigned int             maxItemsPerCell,
                                                          OctreeVec3<Precision>    center,
                                                          Precision                radius,
                                                          unsigned int             threadsNumber) : center(center),
                                                                                                    radius(radius),
                                                                                                    maxItemsPerCell(maxItemsPerCell),
                                                                                                    threadsNumber(threadsNumber) {
        if(threadsNumber == 0) {
            this->threadsNumber = std::thread::hardware_concurrency();
        }
        root = std::make_shared<OctreeCell<LeafDataType, NodeDataType, Precision> >(maxItemsPerCell, center, radius);
    }

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::clear() {
        root = std::make_shared<OctreeCell<LeafDataType, NodeDataType, Precision>>(maxItemsPerCell, center, radius);
        itemsCount = 0;
    }

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::insert(const LeafDataType *item, const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent) {
        if (agent->isItemOverlappingCell(item, center, radius)) {
            if(root->insert(item, agent)) {
                itemsCount++;
            }
        }
    }

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::insert(const LeafDataType *items,
                                                               const unsigned int itemsCount,
                                                               const OctreeAgent<LeafDataType, NodeDataType, Precision> *agentInsert,
                                                               const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision> *agentAdjust,
                                                               bool autoAdjustTree) {

        assert((autoAdjustTree || radius > Precision(0)) && "Radius has to be > 0");

        if (autoAdjustTree && agentAdjust != nullptr && this->itemsCount == 0) {

            OctreeVec3<Precision> max = center + OctreeVec3<Precision>(radius);
            OctreeVec3<Precision> min = center - OctreeVec3<Precision>(radius);

            for (unsigned int i = 0; i < itemsCount; ++i) {
                max = agentAdjust->GetMaxValuesForAutoAdjust(&items[i], max);
                min = agentAdjust->GetMinValuesForAutoAdjust(&items[i], min);
            }
            center = (max + min) / Precision(2);
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

    OctreeTemplate
    template <class T>
    void Octree<LeafDataType, NodeDataType, Precision>::insert(const LeafDataType *items,
                                                               const unsigned int itemsCount,
                                                               const T *agent,
                                                               bool autoAdjustTree) {

        static_assert(std::is_base_of<OctreeAgent<LeafDataType, NodeDataType, Precision>, T>::value, "Agent has wrong class");

        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision>* > (agent);
        if(autoAdjustTree && adj == nullptr) {
            printf("WARNING: To use auto adjust agent has to implement OctreeAgentAutoAdjustExtension\n");
            insert(items, itemsCount, agent, nullptr, false);
        } else {
            insert(items, itemsCount, agent, adj, autoAdjustTree);
        }
    }

    OctreeTemplate
    template <class T>
    void Octree<LeafDataType, NodeDataType, Precision>::insert(const LeafDataType *items,
                                                               const unsigned int itemsCount,
                                                               const T *agent) {

        static_assert(std::is_base_of<OctreeAgent<LeafDataType, NodeDataType, Precision>, T>::value, "Agent has wrong class");
        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision>* > (agent);
        insert(items, itemsCount, agent, adj, adj != nullptr);
    }

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::insert(std::vector<LeafDataType> &items,
                                                               const OctreeAgent<LeafDataType, NodeDataType, Precision> *agentInsert,
                                                               const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision> *agentAdjust,
                                                               bool autoAdjustTree) {

        if (autoAdjustTree && agentAdjust != nullptr && this->itemsCount == 0) {

            OctreeVec3<Precision> max = center + OctreeVec3<Precision>(radius);
            OctreeVec3<Precision> min = center - OctreeVec3<Precision>(radius);

            for (unsigned int i = 0; i < items.size(); ++i) {
                max = agentAdjust->GetMaxValuesForAutoAdjust(&items[i], max);
                min = agentAdjust->GetMinValuesForAutoAdjust(&items[i], min);
            }
            center = (max + min) / Precision(2);
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
                        this->itemsCount++;
                    }
                }
            }
        }
    }

    OctreeTemplate
    template <class T>
    void Octree<LeafDataType, NodeDataType, Precision>::insert(std::vector<LeafDataType> &items, const T *agent, bool autoAdjustTree) {

        static_assert(std::is_base_of<OctreeAgent<LeafDataType, NodeDataType, Precision>, T>::value, "Agent has wrong class");

        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision>* > (agent);
        if(autoAdjustTree && adj == nullptr) {
            printf("WARNING: To use auto adjust agent has to implement OctreeAgentAutoAdjustExtension\n");
            insert(items, agent, nullptr, false);
        } else {
            insert(items, agent, adj, autoAdjustTree);
        }
    }

    OctreeTemplate
    template <class T>
    void Octree<LeafDataType, NodeDataType, Precision>::insert(std::vector<LeafDataType> &items, const T *agent) {

        static_assert(std::is_base_of<OctreeAgent<LeafDataType, NodeDataType, Precision>, T>::value, "Agent has wrong class");
        auto adj = dynamic_cast< const OctreeAgentAutoAdjustExtension<LeafDataType, NodeDataType, Precision>* > (agent);
        insert(items, agent, adj, adj != nullptr);
    }

    OctreeTemplate
    std::string Octree<LeafDataType, NodeDataType, Precision>::getItemPath(LeafDataType *item) const {
        std::string v;
        root->getItemPath(item, v);
        return v;
    }

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::visit(const OctreeVisitor<LeafDataType, NodeDataType, Precision> *visitor) const {
        if (threadsNumber != 1) {
            printf("WARNING: visiting with multiple tree requires OctreeVisitorThreaded\nUsing 1 thread\n");
        }
        visitor->visitRoot(root);
    }

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::visit(const OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision> *visitor) const {
        if (threadsNumber != 1) {

            visitor->visitPreRoot(root);
            if (root->isLeaf()) {
                root->visit(visitor);
            } else {
                visitor->visitPreBranch(root.get(), root->getChilds());
                int from[16][2];
                int to[16][2];
                std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > roots[16][2];
                if(threadsNumber <= 8) {
                    roots[0][0] = root;
                    for (unsigned int i = 0; i < threadsNumber; ++i) {
                        from[i][0] = (8 * i) / threadsNumber;
                        to[i][0] = std::min<int>(8, (8 * (i+1)) / threadsNumber);
                        threads.push_back(std::thread(&Octree::visitThread, this, std::ref(visitor), std::ref(roots[0]), std::ref(from[i]), std::ref(to[i]), 1));
                    }
                    for (unsigned int i = 0; i < threadsNumber; ++i) {
                        threads[i].join();
                    }
                    threads.clear();

                } else {
                    auto childs = root->getChilds();

                    for (int i = 0; i < 8; ++i) {
                        if(childs[i]->isLeaf()) {
                            visitor->visitLeaf(childs[i].get(), childs[i]->getData());
                        } else {
                            visitor->visitPreBranch(childs[i].get(), childs[i]->getChilds());
                        }
                    }

                    for (unsigned int i = 0; i < threadsNumber; ++i) {
                        int fromGeneral = (64 * i) / threadsNumber;
                        int toGeneral = std::min<int>(64, (64 * (i+1)) / threadsNumber);

                        int fromNode = fromGeneral/8;
                        int toNode = std::min(toGeneral/8, 7);
                        if(fromNode == toNode) {
                            from[i][0] = fromGeneral - fromNode * 8;
                            to[i][0] = toGeneral - toNode * 8;
                            roots[i][0] = childs[fromNode];
                            threads.push_back(std::thread(&Octree::visitThread, this, std::ref(visitor), std::ref(roots[i]), std::ref(from[i]), std::ref(to[i]), 1));
                        } else {
                            from[i][0] = fromGeneral - fromNode * 8;
                            to[i][0] = 8;
                            from[i][1] = 0;
                            to[i][1] = toGeneral - toNode * 8;
                            roots[i][0] = childs[fromNode];
                            roots[i][1] = childs[toNode];
                            threads.push_back(std::thread(&Octree::visitThread, this, std::ref(visitor), std::ref(roots[i]), std::ref(from[i]), std::ref(to[i]), 2));
                        }
                    }

                    for (unsigned int i = 0; i < threads.size(); ++i) {
                        threads[i].join();
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

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::insertThread(const OctreeAgent<LeafDataType, NodeDataType, Precision> *agent) {
        unsigned int itemsToBatch = threadsNumber * threadsNumber;

        while (!itemsToAdd.empty()) {
            threadsMutex.lock();
            if (itemsToAdd.empty()) {
                threadsMutex.unlock();
                break;
            }
            std::vector<const LeafDataType *> tempItems;
            tempItems.reserve(itemsToBatch);
            for (unsigned int i = 0; i < itemsToBatch; ++i) {
                if (itemsToAdd.empty()) {
                    break;
                }
                auto item = itemsToAdd[itemsToAdd.size() - 1];
                tempItems.push_back(item);
                itemsToAdd.pop_back();
                this->itemsCount++;
            }
            threadsMutex.unlock();
            for (unsigned int i = 0; i < tempItems.size(); ++i) {
                auto item = tempItems[i];
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

    OctreeTemplate
    void Octree<LeafDataType, NodeDataType, Precision>::visitThread(const OctreeVisitorThreaded<LeafDataType, NodeDataType, Precision> *visitor,
                                                                    std::shared_ptr<OctreeCell<LeafDataType, NodeDataType, Precision> > threadRoots[2],
                                                                    int* from,
                                                                    int* to,
                                                                    int size) const {

        for (int j = 0; j < size; ++j) {
            if(from[j] == to[j])
                continue;
            if(!threadRoots[j]->isLeaf()) {
                auto childs = threadRoots[j]->getChilds();
                for (int i = from[j]; i < to[j]; ++i) {
                    if(childs[i] != nullptr)
                        childs[i]->visit(visitor);
                }
            } else {
                visitor->visitLeaf(threadRoots[j].get(), threadRoots[j]->getData());
            }
        }

    }
}

#endif /* defined(__AKOctree__Octree__) */
