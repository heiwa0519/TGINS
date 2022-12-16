#ifndef JPROGRESS_BAR_HPP
#define JPROGRESS_BAR_HPP

/****************************************************
MIT License
Copyright (c) 2022 ZhengqiaoWang
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/

#include <atomic>
#include <string>
#include <assert.h>
#include <iostream>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <Windows.h>
#elif defined(__linux__)
#include <sys/ioctl.h>
#endif

namespace Joger
{

    namespace ProgressBar
    {
        // If you want to diy the display style, you can edit every sign below
        static constexpr char START_SIGN{'['};
        static constexpr char END_SIGN{']'};
        static constexpr char PROGRESS_SIGN{'#'};
        static constexpr char UNPROGRESS_SIGN{' '};

        // Do not edit these variables if unnecessary
        static constexpr double PROGRESS_WIDTH_RATE{0.8};
        static constexpr unsigned int PROGRESS_WORDS_WIDTH{10};

        /**
         * @brief JProgressBar A simple progressbar for C++11
         *
         */
        class JProgressBar
        {
        public:
            /**
             * @brief Construct a new JProgressBar object
             *
             * @param full_score The maximum value that will be written to the progress bar,
             *        which can be the size of the file to be transferred. Requires greater than 0.
             */
            JProgressBar(double full_score = 100.0,std::string pre_fix="FKF") : m_full_score(full_score)
            {
                assert(full_score > 0);
                getTerminalSize();
                pre_fix_=pre_fix;
                m_full_point = (m_terminal_width - PROGRESS_WORDS_WIDTH) * PROGRESS_WIDTH_RATE;
                if(m_full_point>100){
                    m_full_point=100;
//                    m_full_score=100;
                }
            }

        public:
            /**
             * @brief Update the progress
             *
             * @param cur_score current progress, which can be the size of the file already
             *        transferred. Requires greater than or equal to 0.
             */
            void update(double cur_score)
            {
                assert(cur_score <= m_full_score);
                assert(cur_score >= 0);
                m_cur_score = cur_score;
                m_cur_point = m_cur_score / m_full_score * m_full_point;
                printProgressBar();
            }

            /**
             * @brief End progress bar, marks the end of the current progress bar.
             *
             */
            void end()
            {
                printf("\n");
            }

        private:
            void printProgressBar()
            {
                printf("\r");
                printf("%s: %c", pre_fix_.c_str(),START_SIGN);
                for (unsigned int i = 0; i < m_full_point; ++i)
                {
                    if (i <= m_cur_point)
                    {
                        printf("%c", PROGRESS_SIGN);
                    }
                    else
                    {
                        printf("%c", UNPROGRESS_SIGN);
                    }
                }
                printf("%c %5.1f%%", END_SIGN, m_cur_score / m_full_score * 100);
                fflush(stdout);
            }

            void getTerminalSize()
            {
#if defined(_WIN32)
                CONSOLE_SCREEN_BUFFER_INFO csbi;
                GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
                m_terminal_width = (int)(csbi.dwSize.X);
                m_terminal_height = (int)(csbi.dwSize.Y);
#elif defined(__linux__)
                struct winsize w;
                ioctl(fileno(stdout), TIOCGWINSZ, &w);
                m_terminal_width = (int)(w.ws_col);
                m_terminal_height = (int)(w.ws_row);
#endif // Windows/Linux
            }

        private:
            int m_terminal_width{80}, m_terminal_height{10};

            std::atomic<double> m_cur_score{0};
            double m_full_score;
            unsigned int m_full_point{0};
            unsigned int m_cur_point{0};
            std::string pre_fix_;
        };
    }

}

#endif