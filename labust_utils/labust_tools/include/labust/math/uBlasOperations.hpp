/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef UBLASOPERATIONS_HPP_
#define UBLASOPERATIONS_HPP_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/traits.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <iosfwd>

namespace labust
{
  namespace math
  {
    /**
     * The function computes the skew-symmetric matrix from a given vector.
     * The function operates with vector size 3.
     *
     * \param vec The desired vector of size 3.
     *
     * \tparam in The input matrix type.
     *
     * \return The calculated 3x3 skew symmetric matrix.
     */
    template<class in>
    inline
    boost::numeric::ublas::
    c_matrix<typename boost::numeric::ublas::container_traits<in>::value_type,3,3>
    skewSymm3(const in& vec)
    {
      using namespace boost::numeric::ublas;
      typedef typename container_traits<in>::value_type T;

      c_matrix<T,3,3> mat(zero_matrix<T>(3));
      mat(0,0) = mat(1,1) = mat(2,2) = 0;
      mat(0,1) -= mat(1,0) = vec(2);
      mat(2,0) -= mat(0,2) = vec(1);
      mat(1,2) -= mat(2,1) = vec(0);

      return mat;
    };
    /**
     * The method performs the matrix inversion routine that uses LU decomposition method.
     *
     * \param input The matrix to be inverted.
     * \param inverse The return matrix where the inverse will be calculated.
     *
     * \tparam in The input matrix type.
     * \tparam out The output matrix type.
     *
     * \return If the inversion operation is successful return true, false otherwise.
     */
    template<class in, class out>
    bool inverse(const in& input, out& inverse)
    {
      assert(((input.size1() != 0) && (input.size2() != 0)) && "Inverse: cannot invert 0x0 matrix.");
      assert(((input.size1() == input.size2()) || (input.size1() == 0)) && ("Inverse: only square matrices are supported."));
      typedef boost::numeric::ublas::permutation_matrix<std::size_t> pmatrix;
      typedef typename boost::numeric::ublas::container_traits<in>::value_type value_type;
      typedef boost::numeric::ublas::identity_matrix<value_type> meye;
      // create a working copy of the input
      boost::numeric::ublas::matrix<value_type> A(input);
      // create a permutation matrix for the LU-factorization
      pmatrix pm(A.size1());
      // perform LU-factorization
      int res = boost::numeric::ublas::lu_factorize(A,pm);
      if( res != 0 ) return false;
      // create identity matrix of "inverse"
      inverse.assign(meye(A.size1()));
      // backsubstitute to get the inverse
      boost::numeric::ublas::lu_substitute(A, pm, inverse);
      return true;
    }
    /**
     * The method performs the matrix inversion using the Gauss-Jordan algorithm (Partial Pivot).
     * This inverse is more efficient than the LU decomposition method.
     * Taken from: http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Effective_UBLAS/Matrix_Inversion
     *
     * Usually works 5 times faster then the above inverse.
     *
     * \param input The matrix to be inverted.
     * \param inverse The return matrix where the inverse will be calculated.
     *
     * \tparam in The input matrix type.
     * \tparam out The output matrix type.
     *
     * \return If the inversion operation is successful return true, false otherwise.
     */
    template<class in, class out>
    bool gjinverse(const in& input, out& inverse)
    {
     assert((input.size1() != 0) && (input.size2() != 0) && "Inverse: cannot invert 0x0 matrix.");
     assert(((input.size1() == input.size2()) || (input.size1() == 0)) && "Inverse: only square matrices are supported.");
     typedef typename boost::numeric::ublas::container_traits<in>::value_type value_type;
     typedef typename boost::numeric::ublas::matrix<value_type> matrix;
     typedef typename boost::numeric::ublas::matrix_range<matrix> matrix_range;
     typedef typename boost::numeric::ublas::matrix_row<matrix> row;
     typedef boost::numeric::ublas::identity_matrix<value_type> meye;
     using boost::numeric::ublas::range;
     // Handle 1x1 matrix edge case as general purpose
     // inverter below requires 2x2 to function properly.

     const int size = input.size1();
     bool singular = true;

     if (size == 1)
     {
       if (input(0,0) == 0) return false;
       inverse(0,0) = 1/input(0,0);
       return true;
     }

     // Create an augmented matrix A to invert. Assign the
     // matrix to be inverted to the left hand side and an
     // identity matrix to the right hand side.
     matrix A(size, 2*size);
     matrix_range Aleft(A, range(0, size), range(0, size));
     Aleft = input;
     matrix_range Aright(A, range(0, size), range(size, 2*size));
     Aright = meye(size);
     // Swap rows to eliminate zero diagonal elements.
     for (int k = 0; k < size; k++)
     {
         if ( A(k,k) == 0 ) // XXX: test for "small" instead
         {
             // Find a row(l) to swap with row(k)
             int l = -1;
             for (int i = k+1; i < size; i++)
             {
                 if ( A(i,k) != 0 )
                 {
                     l = i;
                     break;
                 }
             }
             // Swap the rows if found
             if ( l < 0 )
             {
                 std::cerr << "Error:" <<  __FUNCTION__ << ":"
                           << "Input matrix is singular, because cannot find"
                           << " a row to swap while eliminating zero-diagonal.";
                 singular = true;
                 inverse.assign(A);
                 return singular;
             }
             else
             {
                 row rowk(A, k);
                 row rowl(A, l);
                 rowk.swap(rowl);
             }
         }
     }
     // Doing partial pivot
     for (int k = 0; k < size; k++)
     {
         // normalize the current row
         for (int j = k+1; j < 2*size; j++)
             A(k,j) /= A(k,k);
         A(k,k) = 1;
         // normalize other rows
         for (int i = 0; i < size; i++)
         {
             if ( i != k )  // other rows  // FIX: PROBLEM HERE
             {
                 if ( A(i,k) != 0 )
                 {
                     for (int j = k+1; j < 2*size; j++)
                         A(i,j) -= A(k,j) * A(i,k);
                     A(i,k) = 0;
                 }
             }
         }
     }
     inverse.assign(Aright);
     return true;
 }

    /**
     * The function calculates the trace of a matrix.
     *
     * \param matrix The matrix for which we calculate the trace.
     */
    template <class Matrix>
    typename boost::numeric::ublas::container_traits<Matrix>::value_type
    trace(const Matrix& matrix)
    {
    	assert(((matrix.size1() == matrix.size2()) && (matrix.size1() != 0)) && "Trace: only square matrices are supported.");
    	using namespace boost::numeric::ublas;
    	Matrix m(matrix.size1(),matrix.size2());
    	matrix_vector_range<Matrix> diag(m,
    			range (0,matrix.size1()), range (0,matrix.size2()));
    	return sum(diag);
    }
  }
};

namespace boost
{
  namespace numeric
  {
    namespace ublas
    {
      /**
       * The input operator for uBlas constant sized matrices. It uses a workaround
       * with the dynamic sized matrices input operator.
       *
       * \param is The input stream.
       * \param m The constant sized matrix.
       *
       * \tparam E Input stream type.
       * \tparam T Input stream traits.
       * \tparam MT The data type of the constant sized matrix.
       * \tparam N The size1 of the matrix.
       * \tparam M The size2 of the matrix.
       *
       * \return The input stream object.
       */
      template <class E, class T, class MT, std::size_t N, std::size_t M>
      std::basic_istream<E,T>& operator >> (std::basic_istream<E,T>& is, c_matrix<MT,N,M> &m)
      {
        matrix<MT> temp(N,M);
        is>>temp;
        m=temp;

        return is;
      }
      /**
       * The input operator for uBlas constant sized vectors. It uses a workaround
       * with the dynamic sized vectors input operator.
       *
       * \param is The input stream.
       * \param m The constant sized vector.
       *
       * \tparam E Input stream type.
       * \tparam T Input stream traits.
       * \tparam MT The data type of the constant sized vector.
       * \tparam N The vector size.
       *
       * \return The input stream object.
       */
      template <class E, class T, class MT, std::size_t N>
      std::basic_istream<E,T>& operator >> (std::basic_istream<E,T>& is, c_vector<MT,N> &m)
      {
        vector<MT> temp(N);
        is>>temp;
        m=temp;

        return is;
      }
    }
  }
};

/* UBLASOPERATIONS_HPP_ */
#endif
