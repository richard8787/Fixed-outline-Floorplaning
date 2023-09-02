#ifndef MODULE_H
#define MODULE_H

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
using namespace std;

class Terminal
{
public:
    // constructor and destructor
    Terminal(string &name, int x, int y) : _name(name), _x(x), _y(y) {}
    ~Terminal() {}

    // basic access methods
    string getName() const { return _name; }
    int getX() const { return _x; }
    int getY() const { return _y; }

    // set functions
    void setName(string &name) { _name = name; }
    void setX(int x) { _x = x; }
    void setY(int y) { _y = y; }

private:
    string _name; // terminal name
    int _x;       // x coordinate of the terminal
    int _y;       // y coordinate of the terminal
};

class Block
{
public:
    // constructor and destructor
    Block(string &name, int w, int h) : _name(name), _w(w), _h(h), _area(w * h), _rotate(false), _x(0), _y(0), _maxX(w), _maxY(h), _place(false) {}
    ~Block() {}

    // basic access methods
    string getName() const { return _name; };
    int getWidth() const { return _w; }
    int getHeight() const { return _h; }
    int getArea() const { return _area; }
    bool getRotate() const { return _rotate; }
    int getX() const { return _x; }
    int getY() const { return _y; }
    int getMaxX() const { return _maxX; }
    int getMaxY() const { return _maxY; }
    double getCenterX() const { return ((double)_x + (double)_w / 2.0); }
    double getCenterY() const { return ((double)_y + (double)_h / 2.0); }
    bool getPlace() const { return _place; }

    // set functions
    void setName(string &name) { _name = name; }
    void setWidth(int w) { _w = w; }
    void setHeight(int h) { _h = h; }
    void setRotate(bool rotate) { _rotate = rotate; } // need to reset the maxX maxY
    void setRotateToggle() { _rotate = !_rotate; }    // need to reset the maxX maxY
    void setX(int x) { _x = x; }
    void setY(int y) { _y = y; }
    void setMaxX(int maxX) { _maxX = maxX; }
    void setMaxY(int maxY) { _maxY = maxY; }
    void setXYandAutoMax(int x, int y)
    {
        _x = x;
        _y = y;
        setAutoMax();
    }
    void setAutoMax()
    {
        _maxX = _x + _w;
        _maxY = _y + _h;
    }
    void DoRotate()
    {
        swap(_w, _h);
        setAutoMax();
        setRotateToggle();
    }
    void setPlace(bool p) { _place = p; }

private:
    string _name; // block name
    int _w;       // width of the block
    int _h;       // height of the block
    int _area;    // area of the block
    bool _rotate; // rotate or not
    int _x;       // x coordinate of the block
    int _y;       // y coordinate of the block
    int _maxX;    // max x coordinate of the block
    int _maxY;    // max y coordinate of the block
    bool _place;  // be placed or not
};

class Net
{
public:
    // constructor and destructor
    Net(int degree) : _degree(degree) {}
    ~Net() {}

    // basic access methods
    vector<Block *> getBlkList() { return _blkList; }
    vector<Terminal *> getTermList() { return _termList; }
    int getDegree() const { return _degree; }

    // modify methods
    void addBlk(Block *blk) { _blkList.push_back(blk); }
    void addTerm(Terminal *term) { _termList.push_back(term); }
    void setDegree(int degree) { _degree = degree; }

    // other member functions
    double calcHPWL()
    {
        double maxX = 0;
        double minX = 9999999;
        double maxY = 0;
        double minY = 9999999;

        for (int i = 0; i < _blkList.size(); i++)
        {
            maxX = max(maxX, _blkList[i]->getCenterX());
            maxY = max(maxY, _blkList[i]->getCenterY());
            minX = min(minX, _blkList[i]->getCenterX());
            minY = min(minY, _blkList[i]->getCenterY());
        }
        for (int i = 0; i < _termList.size(); i++)
        {
            maxX = max(maxX, (double)_termList[i]->getX());
            maxY = max(maxY, (double)_termList[i]->getY());
            minX = min(minX, (double)_termList[i]->getX());
            minY = min(minY, (double)_termList[i]->getY());
        }

        return (maxX - minX) + (maxY - minY);
    }

private:
    vector<Block *> _blkList;     // list of block the net is connected to
    vector<Terminal *> _termList; // list of terminals the net is connected to
    int _degree;                  // net degree
};

#endif // MODULE_H
