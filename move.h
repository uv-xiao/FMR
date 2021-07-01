#pragma once

#include "types.h"
#include "space.h"
#include "net.h"
#include "parser.h"
#include "config.h"

namespace rt{

class Move{
    public:
    Space &space;
    Move(Space& space): space(space) {};
    void bigMove(Node node);
    std::pair<T2, T2> boundingBox(const std::string& netName, const std::string& exCellName="None");
    std::pair<T2, T2> optimalRegion(const std::string& netName);
    std::map<std::string, stringset> net2neighbors;
    std::map<std::string, stringset> cell2nets;
    std::map<std::string, stringset> net2cells;
    void init();
    void netMove(int direction);
    std::map<stringpair, int> shareCells;
    double computeCongest(std::pair<T2, T2> box, double factor=1.0);
};

}