//
// Created by hw on 12/9/22.
//

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ProcessBar.h"


ProcessBar::ProcessBar(/* args */) {}

ProcessBar::ProcessBar(long totalNum)
{
    totalNum_ = totalNum;
}

ProcessBar::~ProcessBar() {}

void ProcessBar::Init(long totalNum)
{
    totalNum_ = totalNum;
}

void ProcessBar::Progress(long num)
{
    int currentIndex = num  * 100  / step_ / totalNum_;
    for (int j = 0; j < iconMaxNum_; ++j)
    {
        if (j < currentIndex) {
            printf("\033[1;42m%c\033[0m", icon_);
        } else if (j == currentIndex) {
            printf("\033[1;44m%c\033[0m", icon_);
        } else {
            printf("%c", ' ');
        }
    }

    printf("\033[32m %.1f%%\033[0m", 100.0 * num  / totalNum_);
    fflush(stdout);

    usleep(interval_);
    if (num != totalNum_) {
        for (int j = 0; j < (iconMaxNum_ + 3000); ++j)
        {
            printf("\b");
        }
        fflush(stdout);
    } else {
        printf("\n");
    }
}