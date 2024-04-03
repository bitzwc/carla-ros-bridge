#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

using Eigen::VectorXd;
using std::string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/*
  计算多项式的结果
  @coeffs 多项式系数向量，第几个元素表示第几阶
  @x 计算点
*/
double polyeval(const VectorXd &coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
/*
  拟合多项式
  @xvals 输入x向量
  @yvals 输入y向量
  @order 多项式阶数
  @输出 多项式系数向量
*/
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  //A矩阵为[I, X, X^2,..., X^order]
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j)
  {
    for (int i = 0; i < order; ++i)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  //QR分解，Q为正交矩阵，https://blog.csdn.net/u011028771/article/details/78196556
  auto Q = A.householderQr();

  //Ax=b->QRx=b->Rx=QTb，R为上三角矩阵，[R1, 0]T
  auto result = Q.solve(yvals);

  //系数解 order+1维向量
  return result;
}

#endif // HELPERS_H