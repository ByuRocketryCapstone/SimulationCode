#ifndef OPTIMIZER_SOLUTION_H
#define OPTIMIZER_SOLUTION_H


class Solution
{
    public:
    Solution(double kp = 0, double ki = 0, double kd = 0, double score = 0)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->score = score;
    }
    void setGains(double kp, double ki, double kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
    void setScore(double score)
    {
        this->score = score;
    }
    void equals(Solution otherSoln)
    {
        this->kp = otherSoln.kp;
        this->ki = otherSoln.ki;
        this->kd = otherSoln.kd;
        this->score = otherSoln.score;
    }

    double kp, ki, kd, score;

};


#endif //OPTIMIZER_SOLUTION_H