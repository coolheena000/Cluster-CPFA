#ifndef CPFA_LOCALIZATION_H
#define CPFA_LOCALIZATION_H

struct CPFAPose {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    bool valid = false;
};

class CPFALocalization {
public:
    CPFAPose getPose() const {
        return pose_;
    }

    void update(double x, double y, double heading) {
        pose_ = {x, y, heading, true};
    }

    void invalidate() {
        pose_.valid = false;
    }

private:
    CPFAPose pose_;
};

#endif
