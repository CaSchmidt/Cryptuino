/****************************************************************************
** Copyright (c) 2022, Carsten Schmidt. All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
**
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

#ifndef MATRIX_H
#define MATRIX_H

#include <type_traits>
#include <vector>

////// Traits ////////////////////////////////////////////////////////////////

/*
 * NOTE:
 * M-by-N matrices are considered to have M rows and N columns.
 * Element (i,j) refers to the i-th row and the j-th column.
 */

template<typename T>
struct ColumnMajorTraits {
  using  size_type = std::size_t;
  using value_type = T;

  inline static size_type index(const size_type Mrows, const size_type /*Ncols*/,
                                const size_type i, const size_type j)
  {
    return i + j*Mrows;
  }
};

template<typename T>
struct RowMajorTraits {
  using  size_type = std::size_t;
  using value_type = T;

  inline static size_type index(const size_type /*Mrows*/, const size_type Ncols,
                                const size_type i, const size_type j)
  {
    return i*Ncols + j;
  }
};

////// Implementation ////////////////////////////////////////////////////////

template<typename T, typename TraitsT>
class Matrix {
  static_assert( std::is_same_v<T,typename TraitsT::value_type> );

public:
  using traits_type = TraitsT;
  using   size_type = typename traits_type::size_type;
  using  value_type = typename traits_type::value_type;

  ////// Initialization //////////////////////////////////////////////////////

  Matrix() noexcept = default;

  Matrix(const Matrix&) noexcept = default;
  Matrix& operator=(const Matrix&) noexcept = default;

  Matrix(Matrix&&) noexcept = default;
  Matrix& operator=(Matrix&&) noexcept = default;

  ~Matrix() noexcept = default;

  Matrix(const size_type Mrows, const size_type Ncols,
         const value_type init = value_type{0}) noexcept
  {
    resize(Mrows, Ncols, init);
  }

  void clear()
  {
    _data.clear();
    _Mrows = _Ncols = 0;
  }

  bool resize(const size_type Mrows, const size_type Ncols,
              const value_type init = value_type{0}) noexcept
  {
    if( Mrows < 1  ||  Ncols < 1 ) {
      return false;
    }

    clear();

    try {
      _data.resize(Mrows*Ncols, init);
      _Mrows = Mrows;
      _Ncols = Ncols;
    } catch(...) {
      clear();
      return false;
    }

    return true;
  }

  ////// Size Information ////////////////////////////////////////////////////

  inline bool isEmpty() const
  {
    return _data.empty();
  }

  inline size_type columns() const
  {
    return _Ncols;
  }

  inline size_type rows() const
  {
    return _Mrows;
  }

  inline size_type size() const
  {
    return _data.size();
  }

  ////// Data Access /////////////////////////////////////////////////////////

  inline bool isColumnMajor() const
  {
    return index(0, 0) + _Mrows == index(0, 1);
  }

  inline bool isRowMajor() const
  {
    return index(0, 0) + _Ncols == index(1, 0);
  }

  const value_type *data() const
  {
    return _data.data();
  }

  value_type *data()
  {
    return _data.data();
  }

  const value_type *columnData(const size_type j) const
  {
    return _data.data() + index(0, j);
  }

  value_type *columnData(const size_type j)
  {
    return _data.data() + index(0, j);
  }

  const value_type *rowData(const size_type i) const
  {
    return _data.data() + index(i, 0);
  }

  value_type *rowData(const size_type i)
  {
    return _data.data() + index(i, 0);
  }

  ////// Value Access ////////////////////////////////////////////////////////

  inline value_type operator()(const size_type i, const size_type j) const
  {
    return _data[index(i, j)];
  }

  inline value_type& operator()(const size_type i, const size_type j)
  {
    return _data[index(i, j)];
  }

private:
  inline size_type index(const size_type i, const size_type j) const
  {
    return traits_type::index(_Mrows, _Ncols, i, j);
  }

  std::vector<value_type> _data{};
  size_type               _Mrows{};
  size_type               _Ncols{};
};

template<typename T>
using ColMajMatrix = Matrix<T,ColumnMajorTraits<T>>;

template<typename T>
using RowMajMatrix = Matrix<T,RowMajorTraits<T>>;

#endif // MATRIX_H
