#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include "module.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <time.h>
#include <vector>
using namespace std;
class Floorplanner
{
public:
    // constructor and destructor
    Floorplanner(fstream &input_blk, fstream &input_net, double alpha)
    {
        parseInput(input_blk, input_net, alpha);
    }
    ~Floorplanner() {}

    // basic access methods
    int getOutlineX() const { return _outlineX; }
    int getOutlineY() const { return _outlineY; }
    int getBlkNum() const { return _blkNum; }
    int getTermNum() const { return _termNum; }
    int getNetNum() const { return _netNum; }
    int getMaxX() const { return _maxX; }
    int getMaxY() const { return _maxY; }
    int getArea() const { return _area; }
    double getHPWL() const { return _HPWL; }
    double getAreaNorm() const { return _areaNorm; }
    double getHPWLNorm() const { return _HPWLNorm; }
    double getCost() const { return _cost; }
    double getRuntime() const { return (double)clock() / CLOCKS_PER_SEC; }
    double getAlpha() const { return _alpha; }
    double getPenalty() const { return _penalty; }

    // I/O
    void parseInput(fstream &input_blk, fstream &input_net, double alpha)
    {
        // cout << "-----------Parse Start!-----------" << endl;
        string get;
        int id = 0;
        _alpha = alpha;

        input_blk >> get; // outline:

        input_blk >> get; // outline X
        _outlineX = stoi(get);
        input_blk >> get; // outline Y
        _outlineY = stoi(get);

        input_blk >> get; // NumberOfBlock:
        input_blk >> get;
        _blkNum = stoi(get);

        input_blk >> get; // NumberOfTerminal:
        input_blk >> get;
        _termNum = stoi(get);

        for (int i = 0; i < getBlkNum(); i++)
        {
            string blkName, blkW, blkH;
            input_blk >> blkName;
            input_blk >> blkW;
            input_blk >> blkH;
            Block *blk = new Block(blkName, stoi(blkW), stoi(blkH));
            _blkList.push_back(blk);
            _name2Id[blkName] = id++;
        }
        for (int i = 0; i < getTermNum(); i++)
        {
            string termName, termX, termY;
            input_blk >> termName;
            input_blk >> get;
            input_blk >> termX;
            input_blk >> termY;
            Terminal *term = new Terminal(termName, stoi(termX), stoi(termY));
            _termList.push_back(term);
            _name2Id[termName] = id++;
        }

        input_net >> get; // NumNets
        input_net >> get;
        _netNum = stoi(get);
        for (int i = 0; i < getNetNum(); i++)
        {
            string netDegree;
            input_net >> get;
            input_net >> netDegree;
            Net *net = new Net(stoi(netDegree));
            for (int i = 0; i < net->getDegree(); i++)
            {
                string blkTermName;
                input_net >> blkTermName;
                if (_name2Id[blkTermName] < getBlkNum())
                    net->addBlk(_blkList[_name2Id[blkTermName]]);
                else
                    net->addTerm(_termList[_name2Id[blkTermName] - getBlkNum()]);
            }
            _netList.push_back(net);
        }

        _contour.resize(getOutlineX(), 0);
        countOrder();

        // showParserData();
        // cout << "-----------Parse Done!-----------" << endl
        //<< endl;
    }

    void writeResult(fstream &outFile)
    {
        stringstream buff;
        buff << getCost();
        outFile << buff.str() << endl;
        buff.str("");

        buff << getHPWL();
        outFile << buff.str() << endl;
        buff.str("");

        buff << getArea();
        outFile << buff.str() << endl;
        buff.str("");

        buff << getMaxX();
        outFile << buff.str() << " ";
        buff.str("");

        buff << getMaxY();
        outFile << buff.str() << endl;
        buff.str("");

        buff << getRuntime();
        outFile << buff.str() << endl;
        buff.str("");

        for (int i = 0; i < _blkList.size(); i++)
        {
            outFile << _blkList[i]->getName() << " ";
            buff << _blkList[i]->getX();
            outFile << buff.str() << " ";
            buff.str("");
            buff << _blkList[i]->getY();
            outFile << buff.str() << " ";
            buff.str("");
            buff << _blkList[i]->getMaxX();
            outFile << buff.str() << " ";
            buff.str("");
            buff << _blkList[i]->getMaxY();
            outFile << buff.str() << " ";
            buff.str("");
            outFile << endl;
        }
    }

    // floorplan with Simulated annealing
    void floorplan()
    {
        cout << "-----------Floorplan Start!-----------" << endl;

        // initial floorplan by rotateFSB and packingFFD
        initialFP();

        // do simulated annealing
        _penalty = 8; // out of outline penalty
        simulatedAnnealing();

        if (!legal() && _penalty < 500) // if not legal redo the simulated annealing
        {
            _penalty += 20; // increase penalty for those hard to do floorplan
            initialFP();
            simulatedAnnealing();
            cout << "try another Simulated Annealing with higer penalty!" << endl;
        }

        // show the final information for output
        showHPWLMaxXYAreaCost();

        cout << "-----------Floorplan Done!-----------" << endl
             << endl;
    }

    // compacting function
    static bool cmp(Block *a, Block *b)
    {
        return a->getArea() > b->getArea();
    }

    void countOrder()
    {
        _orderedBlkList = _blkList;
        sort(_orderedBlkList.begin(), _orderedBlkList.end(), cmp);
    }

    void putRightOfParent(Block *parent, Block *leftChild)
    {
        // x1=x0+w0 (x0's MaxX)
        int leftChildX = parent->getMaxX();
        leftChild->setXYandAutoMax(leftChildX, *max_element(_contour.begin() + leftChildX, _contour.begin() + leftChildX + leftChild->getWidth()));
        updateContour(leftChild);
    }

    void putUpOfParent(Block *parent, Block *rightChild)
    {
        // x1=x0
        int rightChildX = parent->getX();
        rightChild->setXYandAutoMax(rightChildX, *max_element(_contour.begin() + rightChildX, _contour.begin() + rightChildX + rightChild->getWidth()));
        updateContour(rightChild);
    }

    void updateContour(Block *place) // when place the new block call this
    {
        for (int i = place->getX(); i < place->getMaxX(); i++)
            _contour[i] = place->getMaxY();
    }

    void rotateFSB() // rotate the block for align size with width and height
    {
        for (int i = 0; i < _orderedBlkList.size(); i++)
        {
            if (_outlineX > _outlineY)
            {
                if (_orderedBlkList[i]->getHeight() > _orderedBlkList[i]->getWidth())
                {
                    _orderedBlkList[i]->DoRotate();
                }
            }
            else
            {
                if (_orderedBlkList[i]->getHeight() < _orderedBlkList[i]->getWidth())
                {
                    _orderedBlkList[i]->DoRotate();
                }
            }
        }
    }

    void packingFFD()
    {
        for (int i = 0; i < _contour.size(); i++)
            _contour[i] = 0;

        for (int i = 0; i < _orderedBlkList.size(); i++)
        {
            _orderedBlkList[i]->setXYandAutoMax(0, 0);
            _orderedBlkList[i]->setPlace(false);
        }

        Block *cur = _orderedBlkList[0];
        _orderedBlkList[0]->setXYandAutoMax(0, 0);
        updateContour(_orderedBlkList[0]);

        for (int i = 1; i < _orderedBlkList.size(); i++)
        {
            bool change_row;
            if (_orderedBlkList[i]->getPlace())
            {
                continue;
            }

            if ((cur->getMaxX() + _orderedBlkList[i]->getWidth()) < _outlineX)
            {
                putRightOfParent(cur, _orderedBlkList[i]);
                cur = _orderedBlkList[i];
            }
            else
            {
                change_row = true;
                for (int j = i + 1; j < _orderedBlkList.size(); j++)
                {
                    if (((cur->getMaxX() + _orderedBlkList[j]->getWidth()) < _outlineX) && !_orderedBlkList[j]->getPlace())
                    {
                        putRightOfParent(cur, _orderedBlkList[j]);
                        cur = _orderedBlkList[j];
                        _orderedBlkList[j]->setPlace(true);
                        change_row = false;
                        i--;
                        break;
                    }
                }
                if (change_row)
                {
                    putUpOfParent(_orderedBlkList[0], _orderedBlkList[i]);
                    cur = _orderedBlkList[i];
                }
            }
        }
    }

    void packingArea()
    {
        for (int i = 0; i < _contour.size(); i++)
            _contour[i] = 0;
        for (int i = 0; i < _orderedBlkList.size(); i++)
        {
            _orderedBlkList[i]->setXYandAutoMax(0, 0);
            _orderedBlkList[i]->setPlace(false);
        }
        Block *cur = _orderedBlkList[0];
        _orderedBlkList[0]->setXYandAutoMax(0, 0);
        updateContour(_orderedBlkList[0]);

        for (int i = 1; i < _orderedBlkList.size(); i++)
        {
            if ((cur->getMaxX() + _orderedBlkList[i]->getWidth()) < _outlineX)
            {
                putRightOfParent(cur, _orderedBlkList[i]);
                cur = _orderedBlkList[i];
            }
            else
            {
                putUpOfParent(_orderedBlkList[0], _orderedBlkList[i]);
                cur = _orderedBlkList[i];
            }
        }
    }

    void setNormCoef()
    {
        // count the total HPWL
        countHPWL();

        // find Max X and Y then count area
        countMaxXY();
        countArea();
        _HPWLNorm = getHPWL();
        _areaNorm = getArea();
    }

    void initialFP()
    {
        countOrder();
        rotateFSB();
        packingFFD();
        setNormCoef();
    }

    void simulatedAnnealing()
    {
        // Simulated Annealing
        double T = 1000000000000000000;                                                  // temperature 1000000000000000000
        const double cool = 0.000000000000000000000000000000000000000000000000000000001; // the cool down temperature 0.000000000000000000000000000000000000000000000000000000001
        const double fastT = 0.0001;                                                     // fast temperature 0.0001
        const double refineT = 0.0000001;                                                // gamma update temperature 0.0000001
        double gamma = 0.98;                                                             // gamma 0.98
        int s1, s2;                                                                      // get the random index
        double curCost;                                                                  // the current cost
        double nextCost;                                                                 // the next cost
        double delta;                                                                    // next cost - current cost
        double p;                                                                        // probabiltiy of accept
        mt19937 gen(777);                                                                // random generator
        uniform_real_distribution<double> gen01(0.0, 1.0);                               // generator random from 0 to 1
        double mode, rng;                                                                // the random of mode and probability
        bool modeFlag;                                                                   // chosen mode
        // int counter = 0;                                                                 // count the iteration

        while (T >= cool)
        {
            // cout << "****iteration: " << counter++ << "****" << endl;

            // Get swap and rotate id with random
            s1 = gen() % _orderedBlkList.size();
            s2 = gen() % _orderedBlkList.size();
            // cout << "s1:" << s1 << " s2:" << s2 << endl;

            // set operation mode
            mode = gen01(gen);
            if (mode > 0.1)
                modeFlag = true; // swap
            else
                modeFlag = false; // rotate

            // current cost
            countHPWLMaxXYAreaCost();
            curCost = getCost();
            // cout << "curCost: " << curCost << endl;

            // next operation
            if (modeFlag) // swap
            {
                swap(_orderedBlkList[s1], _orderedBlkList[s2]);
                //  cout << "Swap: " << s1 << " " << s2 << endl;
            }
            else // rotate
            {
                _orderedBlkList[s1]->DoRotate();
                //  cout << "Rotate: " << s1 << endl;
            }

            // next cost
            packingFFD();
            countHPWLMaxXYAreaCost();
            nextCost = getCost();
            // cout << "nextCost:" << nextCost << endl;

            // count accept or not
            delta = nextCost - curCost;
            p = exp(-delta / T);
            rng = gen01(gen);
            if (delta <= 0) // directly accept
            {
                // cout << "Accepth with curCost: " << curCost << " nextCost: " << nextCost << endl;
            }
            else if (p >= rng && T < fastT) // accept with probabiltiy
            {
                // cout << "Accepth with curCost: " << curCost << " nextCost: " << nextCost << endl;
                // cout << "With P: " << p << " RNG: " << rng << endl;
            }
            else // reject
            {
                if (modeFlag)
                    swap(_orderedBlkList[s1], _orderedBlkList[s2]);
                else
                    _orderedBlkList[s1]->DoRotate();
                // cout << "Reject with curCost: " << curCost << " nextCost: " << nextCost << endl;
                packingFFD();
            }

            // cooling
            T *= gamma;
            if (T < refineT)
                gamma = 0.99;
            // cout << endl;
        }
        countHPWLMaxXYAreaCost();
    }

    // count for basic information
    void countHPWL()
    {
        _HPWL = 0;
        for (int i = 0; i < _netList.size(); i++)
        {
            _HPWL += _netList[i]->calcHPWL();
        }
    }

    void countMaxXY()
    {
        _maxX = 0;
        _maxY = 0;
        for (int i = 0; i < _blkList.size(); i++)
        {
            _maxX = max(_maxX, _blkList[i]->getMaxX());
            _maxY = max(_maxY, _blkList[i]->getMaxY());
        }
    }

    void countArea()
    {
        _area = 0;
        _area = _maxX * _maxY;
    }

    void countCost()
    {
        _cost = 0;
        double outY = (double)_maxY / (double)_outlineY;
        if (_maxY < _outlineY)
            outY = 0;
        _cost = getAlpha() * (double)getArea() / getAreaNorm() + (1.0 - getAlpha()) * getHPWL() / getHPWLNorm() + _penalty * outY;
    }

    void countHPWLMaxXYAreaCost()
    {
        // count the total HPWL
        countHPWL();

        // find Max X and Y then count area
        countMaxXY();
        countArea();

        // count the cost
        countCost();
    }

    bool legal()
    {
        return (_outlineX > _maxX) && (_outlineY > _maxY);
    }

    // show the information
    void showParserData()
    {
        cout << "Alpha: " << getAlpha() << endl;
        cout << "Outline: " << getOutlineX() << " " << getOutlineY() << endl;
        cout << "Block Num: " << getBlkNum() << endl;
        cout << "Terminal Num: " << getTermNum() << endl;
        for (int i = 0; i < _blkList.size(); i++)
            cout << "Block: " << _blkList[i]->getName() << " " << _blkList[i]->getWidth() << " " << _blkList[i]->getHeight() << endl;
        for (int i = 0; i < _termList.size(); i++)
            cout << "Terminal: " << _termList[i]->getName() << " " << _termList[i]->getX() << " " << _termList[i]->getY() << endl;
        for (int i = 0; i < _netList.size(); i++)
        {
            cout << "Net" << i << ": ";
            for (int j = 0; j < _netList[i]->getBlkList().size(); j++)
                cout << _netList[i]->getBlkList()[j]->getName() << " ";
            for (int j = 0; j < _netList[i]->getTermList().size(); j++)
                cout << _netList[i]->getTermList()[j]->getName() << " ";
            cout << endl;
        }

        cout << "Contour Size: " << _contour.size() << endl;
    }

    void showBlock(Block *blk)
    {
        cout << "Name: " << blk->getName();
        cout << " x: " << blk->getX() << " y: " << blk->getY() << " maxX: " << blk->getMaxX();
        cout << " maxY: " << blk->getMaxY() << " Width: " << blk->getWidth() << " Height: " << blk->getHeight() << endl;
        cout << "CenterX: " << blk->getCenterX() << " CenterY: " << blk->getCenterY();
        cout << " Area: " << blk->getArea() << " Rotate: " << blk->getRotate() << endl;
    }

    void showContour()
    {
        for (int i = 0; i < _contour.size(); i++)
        {
            cout << _contour[i] << " ";
            if (i % 10 == 9)
                cout << endl;
        }
        cout << endl;
    }

    void showHPWLMaxXYAreaCost()
    {
        cout << "HPWL: " << getHPWL() << endl;
        cout << "Max X: " << getMaxX() << " MaxY: " << getMaxY() << " Area: " << getArea() << endl;
        cout << "Cost: " << getCost() << endl;

        // check legal
        if (legal())
            cout << "The result is Legal!" << endl;
        else
            cout << "The reuslt is NOT Legal!" << endl;
    }

private:
    int _outlineX;
    int _outlineY;
    int _blkNum;
    int _termNum;
    int _netNum;
    int _maxX;
    int _maxY;
    int _area;
    double _HPWL;
    double _areaNorm;
    double _HPWLNorm;
    double _cost;
    double _alpha;
    double _penalty;
    vector<Block *> _blkList;
    vector<Terminal *> _termList;
    vector<Net *> _netList;
    map<string, int> _name2Id;
    vector<int> _contour;
    vector<Block *> _orderedBlkList; // decreasing ordered by area
};
#endif // FLOORPLANNER_H