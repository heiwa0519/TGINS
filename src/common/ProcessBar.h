//
// Created by hw on 12/9/22.
//

#ifndef FUSING_PROCESSBAR_H
#define FUSING_PROCESSBAR_H


class ProcessBar {
private:
    char icon_{' '};
    long totalNum_;
    int step_ = 5;
    int iconMaxNum_ = 45;
    int interval_ = 10;
public:
    ProcessBar(/* args */);
    ProcessBar(long totalNum);
    void Init(long totalNum);
    void Progress(long currentNum);
    ~ProcessBar();
};


#endif //FUSING_PROCESSBAR_H
