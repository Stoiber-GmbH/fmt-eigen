/*
    MIT License

    Copyright (c) 2024 Yunfan REN (renyf@connect.hku.hk)

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
*/

#ifndef FMT_EIGEN_H_
#define FMT_EIGEN_H_

#include <fmt/format.h>

#include <Eigen/Dense>
#include <cstring>

// Utility function to parse integers at compile-time
constexpr int parse_integer(const char *str, std::size_t len) {
  int value = 0;
  for (std::size_t i = 0; i < len; ++i) {
    if (str[i] >= '0' && str[i] <= '9') {
      value = value * 10 + (str[i] - '0');
    }
  }
  return value;
}

template <typename MatrixType>
class MatrixFormatter : fmt::formatter<std::string> {
 private:
  int precision = 6;  // Default precision if not specified

 public:
  // Parse the format string to extract the precision
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) {
    auto it = ctx.begin();
    auto end = ctx.end();

    // Look for '.' to specify precision
    if (it != end && *it == '.') {
      ++it;  // Skip '.'
      auto precision_start = it;
      while (it != end && *it >= '0' && *it <= '9') {
        ++it;
      }
      // Convert to integer
      precision = parse_integer(&(*precision_start),
                                static_cast<size_t>(it - precision_start));
    }

    // Skip to the end of the range
    while (it != end && *it != '}') {
      ++it;
    }

    // Return the iterator after parsing
    return it;
  }

  template <typename Scalar, typename FormatContext>
  constexpr typename std::enable_if<std::is_floating_point<Scalar>::value>::type
  format_element(Scalar value, FormatContext &ctx) const {
    fmt::format_to(ctx.out(), "{:.{}f} ", value, precision);
  }

  template <typename Scalar, typename FormatContext>
  constexpr typename std::enable_if<std::is_integral<Scalar>::value>::type
  format_element(Scalar value, FormatContext &ctx) const {
    fmt::format_to(ctx.out(), "{:d} ", value);
  }

  // Format the matrix with the specified precision
  template <typename FormatContext>
  auto format(const MatrixType &matrix, FormatContext &ctx) const {
    for (int i = 0; i < matrix.rows(); ++i) {
      for (int j = 0; j < matrix.cols(); ++j) {
        format_element(matrix(i, j), ctx);
      }
      if (i != matrix.rows() - 1) {
        fmt::format_to(ctx.out(), "\n");
      }
    }
    return ctx.out();
  }
};

// Specialize the formatter for Eigen::Matrix
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
struct fmt::formatter<
    Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
    : MatrixFormatter<
          Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {};

// Specialize the formatter for Eigen::Transpose
template <typename MatrixType>
struct fmt::formatter<Eigen::Transpose<MatrixType>>
    : MatrixFormatter<Eigen::Transpose<MatrixType>> {};

// Specialize the formatter for Eigen::Block
template <typename Scalar, int Rows, int Cols, int BlockRows, int BlockCols,
          bool InnerPanel>
struct fmt::formatter<Eigen::Block<Eigen::Matrix<Scalar, Rows, Cols>, BlockRows,
                                   BlockCols, InnerPanel>>
    : MatrixFormatter<Eigen::Block<Eigen::Matrix<Scalar, Rows, Cols>, BlockRows,
                                   BlockCols, InnerPanel>> {};

// Specialize the formatter for Eigen::Diagonal
template <typename Scalar, int Rows, int Cols>
struct fmt::formatter<Eigen::Diagonal<Eigen::Matrix<Scalar, Rows, Cols>>>
    : MatrixFormatter<Eigen::Diagonal<Eigen::Matrix<Scalar, Rows, Cols>>> {};


#endif // FMT_EIGEN_H_
