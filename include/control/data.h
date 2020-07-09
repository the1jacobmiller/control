#ifndef DATA_H
#define DATA_H

struct point {
    double x;
    double y;

    point(double x_, double y_) {
        x=x_;
        y=y_;
    }
    point() {}
};

#endif
