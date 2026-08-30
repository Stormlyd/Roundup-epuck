#include "ZooidAlgorithm.h"
using namespace std;
/**
 * Zooid算法 匈牙利算法->解决全局Zooid和Goal的分配
 * @brief AssignmentProblemSolver::AssignmentProblemSolver
 */

AssignmentProblemSolver::AssignmentProblemSolver()
{

}

AssignmentProblemSolver::~AssignmentProblemSolver()
{

}

//vector<vector<double>>为定义一个二维浮点型矩阵，是分配矩阵，n个机器人和m个目标位置，分配结果存储在Assignment一维数组中
double AssignmentProblemSolver::Solve(vector<vector<double> >& DistMatrix,vector<int>& Assignment,TMethod Method)
{
    int N=DistMatrix.size();    // number of columns (tracks)
    int M=DistMatrix[0].size(); // number of rows (measurements)
    //Matrix[i]表示第i行矩阵

    int *assignment		=new int[N];
    double *distIn		=new double[N*M];

    double  cost;
    //初始化数据
    for(int i=0; i<N; i++)
    {
        for(int j=0; j<M; j++)
        {
            distIn[i+N*j] = DistMatrix[i][j];
        }
    }
    //将二维vector转换为一维数组，按列优先存储

    //选取计算他们的计算时间复杂度不同
    switch(Method)
    {
        case optimal: assignmentoptimal(assignment, &cost, distIn, N, M); break;  //最优分配
        case many_forbidden_assignments: assignmentsuboptimal1(assignment, &cost, distIn, N, M); break;
        case without_forbidden_assignments: assignmentsuboptimal2(assignment, &cost, distIn, N, M); break;
    }

    //结果
    Assignment.clear();
    for(int x=0; x<N; x++)
    {
        Assignment.push_back(assignment[x]);
    }

    delete[] assignment;
    delete[] distIn;
    return cost;
}

// --------------------------------------------------------------------------
// 使用Munkres算法计算最优分配(总成本最小)。
// --------------------------------------------------------------------------
void AssignmentProblemSolver::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
    double *distMatrix;
    double *distMatrixTemp;
    double *distMatrixEnd;
    double *columnEnd;
    double  value;
    double  minValue;

    bool *coveredColumns;
    bool *coveredRows;
    bool *starMatrix;
    bool *newStarMatrix;
    bool *primeMatrix;

    int nOfElements;
    int minDim;
    int row;
    int col;

    // 数据初始化
    *cost = 0;
    for(row=0; row<nOfRows; row++)
    {
        assignment[row] = -1.0;
    }

    nOfElements   = nOfRows * nOfColumns;//总元素数量

    distMatrix    = (double *)malloc(nOfElements * sizeof(double));//分配内存

    distMatrixEnd = distMatrix + nOfElements;//指向最后一个元素 首地址+偏移量

    //生成距离矩阵，并校验矩阵元素的
    for(row=0; row<nOfElements; row++)
    {
        value = distMatrixIn[row];
        if(value < 0)
        {
            cout << "所有的元素素必须大于0." << endl;
        }
        distMatrix[row] = value;
    }

    //内存分配
    coveredColumns = (bool *)calloc(nOfColumns,  sizeof(bool));
    coveredRows    = (bool *)calloc(nOfRows,     sizeof(bool));
    starMatrix     = (bool *)calloc(nOfElements, sizeof(bool));
    primeMatrix    = (bool *)calloc(nOfElements, sizeof(bool));
    newStarMatrix  = (bool *)calloc(nOfElements, sizeof(bool)); /* used in step4 */

    //准备阶段
    if(nOfRows <= nOfColumns)
    {
        minDim = nOfRows;
        for(row=0; row<nOfRows; row++)
        {
            //找到最小值
            distMatrixTemp = distMatrix + row;
            minValue = *distMatrixTemp;
            distMatrixTemp += nOfRows;
            while(distMatrixTemp < distMatrixEnd)
            {
                value = *distMatrixTemp;
                if(value < minValue)
                {
                    minValue = value;
                }
                distMatrixTemp += nOfRows;
            }

            //从行的每个元素中减去最小的元素
            distMatrixTemp = distMatrix + row;
            while(distMatrixTemp < distMatrixEnd)
            {
                *distMatrixTemp -= minValue;
                distMatrixTemp += nOfRows;
            }
        }


        for(row=0; row<nOfRows; row++)
        {
            for(col=0; col<nOfColumns; col++)
            {
                if(distMatrix[row + nOfRows*col] == 0)
                {
                    if(!coveredColumns[col])
                    {
                        starMatrix[row + nOfRows*col] = true;
                        coveredColumns[col]           = true;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        minDim = nOfColumns;
        for(col=0; col<nOfColumns; col++)
        {
            distMatrixTemp = distMatrix     + nOfRows*col;
            columnEnd      = distMatrixTemp + nOfRows;
            minValue = *distMatrixTemp++;
            while(distMatrixTemp < columnEnd)
            {
                value = *distMatrixTemp++;
                if(value < minValue)
                {
                    minValue = value;
                }
            }

            distMatrixTemp = distMatrix + nOfRows*col;
            while(distMatrixTemp < columnEnd)
            {
                *distMatrixTemp++ -= minValue;
            }
        }

        for(col=0; col<nOfColumns; col++)
        {
            for(row=0; row<nOfRows; row++)
            {
                if(distMatrix[row + nOfRows*col] == 0)
                {
                    if(!coveredRows[row])
                    {
                        starMatrix[row + nOfRows*col] = true;
                        coveredColumns[col]           = true;
                        coveredRows[row]              = true;
                        break;
                    }
                }
            }
        }

        for(row=0; row<nOfRows; row++)
        {
            coveredRows[row] = false;
        }
    }


    step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

    //计算成本并删除无效的分配
    computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);

    //内存释放
    free(distMatrix);
    free(coveredColumns);
    free(coveredRows);
    free(starMatrix);
    free(primeMatrix);
    free(newStarMatrix);
    return;
}

// --------------------------------------------------------------------------
// 建立赋值向量
// --------------------------------------------------------------------------
void AssignmentProblemSolver::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
    int row, col;
    for(row=0; row<nOfRows; row++)
    {
        for(col=0; col<nOfColumns; col++)
        {
            if(starMatrix[row + nOfRows*col])
            {
                assignment[row] = col;
                break;
            }
        }
    }
}

// --------------------------------------------------------------------------
// 计算价值
// --------------------------------------------------------------------------
void AssignmentProblemSolver::computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
{
    int row, col;
    for(row=0; row<nOfRows; row++)
    {
        col = assignment[row];
        if(col >= 0)
        {
            *cost += distMatrix[row + nOfRows*col];
        }
    }
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void AssignmentProblemSolver::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool *starMatrixTemp, *columnEnd;
    int col;

    for(col=0; col<nOfColumns; col++)
    {
        starMatrixTemp = starMatrix     + nOfRows*col;
        columnEnd      = starMatrixTemp + nOfRows;
        while(starMatrixTemp < columnEnd)
        {
            if(*starMatrixTemp++)
            {
                coveredColumns[col] = true;
                break;
            }
        }
    }

    step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void AssignmentProblemSolver::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    int col, nOfCoveredColumns;

    nOfCoveredColumns = 0;
    for(col=0; col<nOfColumns; col++)
    {
        if(coveredColumns[col])
        {
            nOfCoveredColumns++;
        }
    }
    if(nOfCoveredColumns == minDim)
    {
        buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
    }
    else
    {
        step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void AssignmentProblemSolver::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool zerosFound;
    int row, col, starCol;
    zerosFound = true;
    while(zerosFound)
    {
        zerosFound = false;
        for(col=0; col<nOfColumns; col++)
        {
            if(!coveredColumns[col])
            {
                for(row=0; row<nOfRows; row++)
                {
                    if((!coveredRows[row]) && (distMatrix[row + nOfRows*col] == 0))
                    {

                        primeMatrix[row + nOfRows*col] = true;

                        for(starCol=0; starCol<nOfColumns; starCol++)
                            if(starMatrix[row + nOfRows*starCol])
                            {
                                break;
                            }
                            if(starCol == nOfColumns)
                            {
                                step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
                                return;
                            }
                            else
                            {
                                coveredRows[row]        = true;
                                coveredColumns[starCol] = false;
                                zerosFound              = true;
                                break;
                            }
                    }
                }
            }
        }
    }

    step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void AssignmentProblemSolver::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
    int n, starRow, starCol, primeRow, primeCol;
    int nOfElements = nOfRows*nOfColumns;

    for(n=0; n<nOfElements; n++)
    {
        newStarMatrix[n] = starMatrix[n];
    }

    newStarMatrix[row + nOfRows*col] = true;

    starCol = col;
    for(starRow=0; starRow<nOfRows; starRow++)
    {
        if(starMatrix[starRow + nOfRows*starCol])
        {
            break;
        }
    }
    while(starRow<nOfRows)
    {

        newStarMatrix[starRow + nOfRows*starCol] = false;

        primeRow = starRow;
        for(primeCol=0; primeCol<nOfColumns; primeCol++)
        {
            if(primeMatrix[primeRow + nOfRows*primeCol])
            {
                break;
            }
        }

        newStarMatrix[primeRow + nOfRows*primeCol] = true;

        starCol = primeCol;
        for(starRow=0; starRow<nOfRows; starRow++)
        {
            if(starMatrix[starRow + nOfRows*starCol])
            {
                break;
            }
        }
    }

    for(n=0; n<nOfElements; n++)
    {
        primeMatrix[n] = false;
        starMatrix[n]  = newStarMatrix[n];
    }
    for(n=0; n<nOfRows; n++)
    {
        coveredRows[n] = false;
    }

    step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void AssignmentProblemSolver::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    double h, value;
    int row, col;

    h = DBL_MAX;
    for(row=0; row<nOfRows; row++)
    {
        if(!coveredRows[row])
        {
            for(col=0; col<nOfColumns; col++)
            {
                if(!coveredColumns[col])
                {
                    value = distMatrix[row + nOfRows*col];
                    if(value < h)
                    {
                        h = value;
                    }
                }
            }
        }
    }

    for(row=0; row<nOfRows; row++)
    {
        if(coveredRows[row])
        {
            for(col=0; col<nOfColumns; col++)
            {
                distMatrix[row + nOfRows*col] += h;
            }
        }
    }

    for(col=0; col<nOfColumns; col++)
    {
        if(!coveredColumns[col])
        {
            for(row=0; row<nOfRows; row++)
            {
                distMatrix[row + nOfRows*col] -= h;
            }
        }
    }

    step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


// --------------------------------------------------------------------------
// 计算次优解。适用于没有禁止任务的情况。
// --------------------------------------------------------------------------
void AssignmentProblemSolver::assignmentsuboptimal2(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
    int n, row, col, tmpRow, tmpCol, nOfElements;
    double value, minValue, *distMatrix;

    nOfElements   = nOfRows * nOfColumns;
    distMatrix    = (double *)malloc(nOfElements * sizeof(double));
    for(n=0; n<nOfElements; n++)
    {
        distMatrix[n] = distMatrixIn[n];
    }

    *cost = 0;
    for(row=0; row<nOfRows; row++)
    {
        assignment[row] = -1.0;
    }

    while(true)
    {
        minValue = DBL_MAX;
        for(row=0; row<nOfRows; row++)
            for(col=0; col<nOfColumns; col++)
            {
                value = distMatrix[row + nOfRows*col];
                if(value!=DBL_MAX && (value < minValue))
                {
                    minValue = value;
                    tmpRow   = row;
                    tmpCol   = col;
                }
            }

            if(minValue!=DBL_MAX)
            {
                assignment[tmpRow] = tmpCol;
                *cost += minValue;
                for(n=0; n<nOfRows; n++)
                {
                    distMatrix[n + nOfRows*tmpCol] = DBL_MAX;
                }
                for(n=0; n<nOfColumns; n++)
                {
                    distMatrix[tmpRow + nOfRows*n] = DBL_MAX;
                }
            }
            else
                break;

    }

    free(distMatrix);
}

// --------------------------------------------------------------------------
// 计算次优解。适用于有许多被禁止的任务的情况。
// --------------------------------------------------------------------------
void AssignmentProblemSolver::assignmentsuboptimal1(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
    bool infiniteValueFound, finiteValueFound, repeatSteps, allSinglyValidated, singleValidationFound;
    int n, row, col, tmpRow, tmpCol, nOfElements;
    int *nOfValidObservations, *nOfValidTracks;
    double value, minValue, *distMatrix;

    nOfElements   = nOfRows * nOfColumns;
    distMatrix    = (double *)malloc(nOfElements * sizeof(double));
    for(n=0; n<nOfElements; n++)
    {
        distMatrix[n] = distMatrixIn[n];
    }

    *cost = 0;

    for(row=0; row<nOfRows; row++)
    {
        assignment[row] = -1.0;
    }

    nOfValidObservations  = (int *)calloc(nOfRows,    sizeof(int));
    nOfValidTracks        = (int *)calloc(nOfColumns, sizeof(int));


    infiniteValueFound = false;
    finiteValueFound  = false;
    for(row=0; row<nOfRows; row++)
    {
        for(col=0; col<nOfColumns; col++)
        {
            if(distMatrix[row + nOfRows*col]!=DBL_MAX)
            {
                nOfValidTracks[col]       += 1;
                nOfValidObservations[row] += 1;
                finiteValueFound = true;
            }
            else
                infiniteValueFound = true;
        }
    }

    if(infiniteValueFound)
    {
        if(!finiteValueFound)
        {
            return;
        }
        repeatSteps = true;

        while(repeatSteps)
        {
            repeatSteps = false;

            for(col=0; col<nOfColumns; col++)
            {
                singleValidationFound = false;
                for(row=0; row<nOfRows; row++)
                    if(distMatrix[row + nOfRows*col]!=DBL_MAX && (nOfValidObservations[row] == 1))
                    {
                        singleValidationFound = true;
                        break;
                    }

                    if(singleValidationFound)
                    {
                        for(row=0; row<nOfRows; row++)
                            if((nOfValidObservations[row] > 1) && distMatrix[row + nOfRows*col]!=DBL_MAX)
                            {
                                distMatrix[row + nOfRows*col] = DBL_MAX;
                                nOfValidObservations[row] -= 1;
                                nOfValidTracks[col]       -= 1;
                                repeatSteps = true;
                            }
                    }
            }


            if(nOfColumns > 1)
            {
                for(row=0; row<nOfRows; row++)
                {
                    singleValidationFound = false;
                    for(col=0; col<nOfColumns; col++)
                    {
                        if(distMatrix[row + nOfRows*col]!=DBL_MAX && (nOfValidTracks[col] == 1))
                        {
                            singleValidationFound = true;
                            break;
                        }
                    }

                    if(singleValidationFound)
                    {
                        for(col=0; col<nOfColumns; col++)
                        {
                            if((nOfValidTracks[col] > 1) && distMatrix[row + nOfRows*col]!=DBL_MAX)
                            {
                                distMatrix[row + nOfRows*col] = DBL_MAX;
                                nOfValidObservations[row] -= 1;
                                nOfValidTracks[col]       -= 1;
                                repeatSteps = true;
                            }
                        }
                    }
                }
            }
        }


        for(row=0; row<nOfRows; row++)
        {
            if(nOfValidObservations[row] > 1)
            {
                allSinglyValidated = true;
                minValue = DBL_MAX;
                for(col=0; col<nOfColumns; col++)
                {
                    value = distMatrix[row + nOfRows*col];
                    if(value!=DBL_MAX)
                    {
                        if(nOfValidTracks[col] > 1)
                        {
                            allSinglyValidated = false;
                            break;
                        }
                        else if((nOfValidTracks[col] == 1) && (value < minValue))
                        {
                            tmpCol   = col;
                            minValue = value;
                        }
                    }
                }

                if(allSinglyValidated)
                {
                    assignment[row] = tmpCol;
                    *cost += minValue;
                    for(n=0; n<nOfRows; n++)
                    {
                        distMatrix[n + nOfRows*tmpCol] = DBL_MAX;
                    }
                    for(n=0; n<nOfColumns; n++)
                    {
                        distMatrix[row + nOfRows*n] = DBL_MAX;
                    }
                }
            }
        }

        //对于每个多重验证的观测，只验证单一验证轨道，选择最小距离的轨道
        for(col=0; col<nOfColumns; col++)
        {
            if(nOfValidTracks[col] > 1)
            {
                allSinglyValidated = true;
                minValue = DBL_MAX;
                for(row=0; row<nOfRows; row++)
                {
                    value = distMatrix[row + nOfRows*col];
                    if(value!=DBL_MAX)
                    {
                        if(nOfValidObservations[row] > 1)
                        {
                            allSinglyValidated = false;
                            break;
                        }
                        else if((nOfValidObservations[row] == 1) && (value < minValue))
                        {
                            tmpRow   = row;
                            minValue = value;
                        }
                    }
                }

                if(allSinglyValidated)
                {
                    assignment[tmpRow] = col;
                    *cost += minValue;
                    for(n=0; n<nOfRows; n++)
                        distMatrix[n + nOfRows*col] = DBL_MAX;
                    for(n=0; n<nOfColumns; n++)
                        distMatrix[tmpRow + nOfRows*n] = DBL_MAX;
                }
            }
        }
    }

    while(true)
    {

        minValue = DBL_MAX;
        for(row=0; row<nOfRows; row++)
            for(col=0; col<nOfColumns; col++)
            {
                value = distMatrix[row + nOfRows*col];
                if(value!=DBL_MAX && (value < minValue))
                {
                    minValue = value;
                    tmpRow   = row;
                    tmpCol   = col;
                }
            }

            if(minValue!=DBL_MAX)
            {
                assignment[tmpRow] = tmpCol;
                *cost += minValue;
                for(n=0; n<nOfRows; n++)
                    distMatrix[n + nOfRows*tmpCol] = DBL_MAX;
                for(n=0; n<nOfColumns; n++)
                    distMatrix[tmpRow + nOfRows*n] = DBL_MAX;
            }
            else
                break;

    }

    free(nOfValidObservations);
    free(nOfValidTracks);
}

/*
// --------------------------------------------------------------------------
// 算法测试
// --------------------------------------------------------------------------
void main(void)
{
    int N=8; // tracks
    int M=9; // detects

    srand (time(NULL));

    vector<vector<double>>Cost(N,vector<double>(M));

    for(int i=0; i<N; i++)
    {
        for(int j=0; j<M; j++)
        {
            Cost[i][j] = (double)(rand()%1000)/1000.0;
            std::cout << Cost[i][j] << "\t";
        }
        std::cout << std::endl;
    }
    AssignmentProblemSolver APS;
    vector<int> Assignment;

    cout << APS.Solve(Cost,Assignment) << endl;

    //输出
    for(int x=0; x<N; x++)
    {
        std::cout << x << ":" << Assignment[x] << "\t";
    }
    getchar();
}
*/
// --------------------------------------------------------------------------


