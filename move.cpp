#include "move.h"
#include "math.h"

namespace rt{

void Move::init() {
    for (auto& cell:space.chip.cellInss) {
        cell2nets.insert({cell.second.insName, rt::stringset()});
    }
    for (auto& net: space.chip.nets) {
        auto& netName = net.second.netName;
        net2cells.insert({netName, rt::stringset()});
        net2neighbors.insert({netName, rt::stringset()});
        for (auto& pin: net.second.pins) {
            auto& cellName = pin[0];
            cell2nets[cellName].insert(netName);
            net2cells[netName].insert(cellName);
        }
    }
    for (auto& netSet: cell2nets) {
        for (const std::string& net1: netSet.second)
            for (const std::string& net2: netSet.second)
                if (net1 != net2) {
                    auto iter = shareCells.find(stringpair(net1, net2));
                    if(iter!=shareCells.end()) {
                        iter->second = 1;
                        net2neighbors[net1].insert(net2);
                    } else {
                        iter->second += 1;
                    }
                }
    }
}

std::pair<T2, T2> Move::boundingBox(const std::string& netName, const std::string& exCellName) {
    auto net = space.chip.nets[netName];
    int min_x = INFINITY, min_y = INFINITY, max_x = 0, max_y = 0;
    for (auto& pin: net.pins) {
        auto cellName = pin[0];
        if (cellName == exCellName)
            continue;

        auto cell = space.chip.cellInss[cellName];

        min_x = std::min(min_x, cell.rowIdx);
        max_x = std::max(max_x, cell.rowIdx);

        min_y = std::min(min_y, cell.colIdx);
        max_y = std::max(max_y, cell.colIdx);
    }
    return std::make_pair(T2{min_x, max_x}, T2{min_y, max_y});
}

std::pair<T2, T2> Move::optimalRegion(const std::string& cellName) {
    std::vector<int> xs;
    std::vector<int> ys;

    for (auto& netName: cell2nets[cellName]) {
        auto box = boundingBox(netName, cellName);
        int lx = std::get<0>(box).first;
        int ux = std::get<1>(box).second;
        int ly = std::get<0>(box).first;
        int uy = std::get<1>(box).second;
        xs.push_back(lx);
        xs.push_back(ux);
        ys.push_back(ly);
        ys.push_back(uy);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::pair<T2, T2> region = std::make_pair(T2{xs[xs.size()/2-1], xs[xs.size()/2]},\
                                 T2{ys[ys.size()/2-1], ys[ys.size()/2]});
    return region;
}

double Move::computeCongest(std::pair<T2, T2> box, double factor) {
    int lx = std::get<0>(box).first, ux = std::get<0>(box).second;
    int ly = std::get<1>(box).first, uy = std::get<1>(box).second;
    double congest;
    for (int x = lx; x <= ux; x++)
        for (int y = ly; y <= uy; y++) {
            congest += space._sumLayer(T2{x, y}, Space::_getSupplyOnGrid) - factor * space._sumLayer(T2{x, y}, Space::_getDemandOnGrid);
        }
    return congest;
}

void Move::netMove(int direction) {
    std::map<std::string, int> center;
    std::map<std::string, int> newCenter;
    std::map<std::string, double> congest;
    std::priority_queue<std::pair<double, std::string> > bestCongestNetQueue;  // (bestCongest, net)
    auto boxCenter = [&](std::pair<T2, T2> box, int direction) {
        int mid_x = (std::get<0>(box).first + std::get<0>(box).second)/2;
        int mid_y = (std::get<1>(box).first + std::get<1>(box).second)/2;
        return direction == 0 ? mid_x : mid_y;
    };
    
    for (auto& net: space.chip.nets) {
        auto& netName = net.second.netName;
        auto box = boundingBox(netName);
        center.insert({netName, boxCenter(box, direction)});
        congest.insert({netName, computeCongest(box)});
    }
    for (auto& net: space.chip.nets) {
        auto& netName = net.second.netName;
        newCenter[netName] = center[netName];
        if (congest[netName] < 0) {
            double bestCongest = congest[netName];
            for (auto net2: net2neighbors[netName])
                bestCongest = std::max(bestCongest, congest[net2]);
            if (bestCongest < 0 || bestCongest == congest[netName])
                continue;
            bestCongestNetQueue.push(std::make_pair(bestCongest, netName));
            for (auto net2: net2neighbors[netName]) {
                newCenter[netName] += (center[net2] - center[netName]) *\
                 (congest[net2] - congest[netName])/(bestCongest - congest[netName]) *\
                  shareCells[stringpair(netName, net2)] / net2cells.size();
            }
        }
    }
    
    auto computeBestNet2Move = [&](int max_n, double area_proportion) {
        stringset net2Move;
        auto& bound = space.chip.gGridBoundaryIdx;
        int area = 0, max_area = (bound[1] - bound[0]) * (bound[3] - bound[2]) * area_proportion;
        while(net2Move.size() < max_n) {
            auto netName = bestCongestNetQueue.top().second;
            auto box = boundingBox(netName);
            area += (std::get<0>(box).second - std::get<0>(box).first) * (std::get<1>(box).second - std::get<1>(box).first);
            if (area > max_area)
                break;
            net2Move.insert(netName);
            bestCongestNetQueue.pop();
        }
        return net2Move;
    };
    
    auto computeCell2Move = [&](stringset& net2Move) {
        stringset cell2Move;
        for (auto& netName: net2Move) {
            for (auto& cell: net2cells[netName]) {
                cell2Move.insert(cell);
            }
        }
        return cell2Move;
    };

    auto net2Move = computeBestNet2Move(3, 0.3);
    auto cell2Move = computeCell2Move(net2Move);
    std::map<std::string, T2> moveRegion;
    for (auto& cellName: cell2Move) {
        auto& cell = space.cellInss[cellName];
        int coord_cell = (direction == 0) ? cell.rowIdx : cell.colIdx;
        std::vector<int> moveCandidate;
        for (auto& netName: cell2nets[cellName]) {
            if (net2Move.find(netName) != net2Move.end())
                moveCandidate.push_back(coord_cell + newCenter[netName] - center[netName]);
        }
        int coord_max = 0, coord_min = INFINITY;
        for (auto cand: moveCandidate) {
            coord_max = std::max(coord_max, cand);
            coord_min = std::min(coord_min, cand);
        }
        int middle = (coord_min + coord_max)/2;
        int l = coord_min, r = coord_max;
        for (auto cand: moveCandidate) {
            if (cand > middle)
                r = std::min(r, cand);
            if (cand < middle)
                l = std::max(l, cand);
        }
        T2 c1, c2;
        if (direction == 0) {
            c1 = T2{l, cell.colIdx};
            c2 = T2{r, cell.colIdx};
        } else {
            c1 = T2{cell.rowIdx, l};
            c2 = T2{cell.rowIdx, r};
        }
        if (computeCongest(std::make_pair(c1, c1)) > computeCongest(std::make_pair(c2,c2)))
            moveRegion.insert({cellName, c1});
        else moveRegion.insert({cellName, c2});
    }
    for (auto pair: moveRegion)
        space._moveCell(pair.first, pair.second);
    
    stringset affectedNets;
    for (auto& cellName: cell2Move) {
        for (auto& netName: cell2nets[cellName])
            affectedNets.insert(netName);
    }
    for (auto& netName: affectedNets) {
        // space.nets[netName]->reroute();
    }
}



}