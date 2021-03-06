#pragma once

#include "bkioctomap.h"
#include <stdlib.h>
using namespace std;

namespace semantic_bki {

	/*
     * @brief Bayesian Generalized Kernel Inference on Bernoulli distribution
     * @param dim dimension of data (2, 3, etc.)
     * @param T data type (float, double, etc.)
     * @ref Nonparametric Bayesian inference on multivariate exponential families
     */
    template<int dim, typename T>
    class SemanticBKInference {
    public:
        /// Eigen matrix type for training and test data and kernel
        using MatrixXType = Eigen::Matrix<T, -1, dim, Eigen::RowMajor>;
        using MatrixKType = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
        using MatrixDKType = Eigen::Matrix<T, -1, 1>;
        using MatrixYType = Eigen::Matrix<T, -1, -1>;

        SemanticBKInference(int nc, T sf2, T ell) : nc(nc), sf2(sf2), ell(ell), trained(false) { }

        /*
         * @brief Fit BGK Model
         * @param x input vector (3N, row major)
         * @param y target vector (N)
         */
        void train(const std::vector<T> &x, const std::vector<std::vector<T> > &y) {
            assert(x.size() % dim == 0 && (int) (x.size() / dim) == y.size());
            MatrixXType _x = Eigen::Map<const MatrixXType>(x.data(), x.size() / dim, dim);
            MatrixYType _y(y.size(), this->nc); // data, num rows, num cols
            for (size_t idx = 0; idx < y.size(); ++idx) {
              _y.row(idx) = Eigen::VectorXf::Map(y[idx].data(), y[idx].size());
            }
            
            this->y_vec = y;
            // std::cout << "x " << _x.size() << std::endl;
            // std::cout << "yvec size train " << this->y_vec.size() << std::endl;
            train(_x, _y);
        }

        /*
         * @brief Fit BGK Model
         * @param x input matrix (NX3)
         * @param y target matrix (NXnc) where nc is number of classes
         */
        void train(const MatrixXType &x, const MatrixYType &y) {
            this->x = MatrixXType(x);
            this->y = MatrixYType(y);
            trained = true;
        }

       
      // void predict(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars) {
      //     assert(xs.size() % dim == 0);
      //     MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
      //     assert(trained == true);
      //     MatrixKType Ks;

      //     covSparse(_xs, x, Ks);
          
      //     ybars.resize(_xs.rows());
      //     for (int r = 0; r < _xs.rows(); ++r)
      //       ybars[r].resize(nc);
            
      //       MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1);
            
      //       for (int k = 0; k < nc; ++k) {
      //         for (int i = 0; i < y_vec.size(); ++i) {
      //           if (y_vec[i] == k)
      //             _y_vec(i, 0) = 1;
      //           else
      //             _y_vec(i, 0) = 0;
      //         }
            
      //       MatrixYType _ybar;
      //       _ybar = (Ks * _y_vec);
            
      //       for (int r = 0; r < _ybar.rows(); ++r)
      //         ybars[r][k] = _ybar(r, 0);
      //     }
      // }

      // void predict_csm(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars) {
      //     assert(xs.size() % dim == 0);
      //     MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
      //     assert(trained == true);
      //     MatrixKType Ks;

      //     covCountingSensorModel(_xs, x, Ks);
          
      //     ybars.resize(_xs.rows());
      //     for (int r = 0; r < _xs.rows(); ++r)
      //       ybars[r].resize(nc);

      //       MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1);
      //       for (int k = 0; k < nc; ++k) {
      //         for (int i = 0; i < y_vec.size(); ++i) {
      //           if (y_vec[i] == k)
      //             _y_vec(i, 0) = 1;
      //           else
      //             _y_vec(i, 0) = 0;
      //         }
            
      //       MatrixYType _ybar;
      //       _ybar = (Ks * _y_vec);
            
      //       for (int r = 0; r < _ybar.rows(); ++r)
      //         ybars[r][k] = _ybar(r, 0);
      //     }
      // }

      // Updates ybars to be samples per class
      // TODO: THIS IS TAKING TOO LONG NOT WOKRING< NEED TO UNDERSTAND
      void predict_softmax(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars, int N) {
        assert(xs.size() % dim == 0);
        MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
        assert(trained == true);
        MatrixKType Ks;

        covSparse(_xs, this->x, Ks);

        ybars.resize(_xs.rows(), vector<T>(nc, 0));

        // std::cout << "num rows " << _xs.rows() << std::endl;
        // std::cout << "yvec size for r " << this->y_vec.size() << std::endl;
        // for (int r = 0; r < _xs.rows(); ++r) {
        //   std::cout << "STarting loop again with r " << r << std::endl;

        MatrixYType _y_vec = MatrixYType::Zero(this->y_vec.size(), this->nc); // data, num rows, num cols
        for (size_t idx = 0; idx < this->y_vec.size(); ++idx) {
          _y_vec.row(idx) = Eigen::VectorXf::Map(this->y_vec[idx].data(), this->y_vec[idx].size());
        }
        _y_vec = _y_vec * N;

        // for (int y_idx = 0; y_idx < this->y_vec.size(); ++y_idx) {
        //   // cumulative distribution probability for each class
        //   vector<T> sample_prob = this->y_vec[y_idx];

        //   for (int j = 1; j < nc; ++j) {
        //     sample_prob[j] += sample_prob[j - 1];
        //   }

        //   for (int i = 0; i < N; ++i) {
        //     float p = (rand() % 100) * 0.01;
        //     for (int k = 0; k < nc; ++k) {
        //       if (k == 0 && p <= sample_prob[k]) {
        //         _y_vec(y_idx, k) += 1;
        //       } 
        //       if (k > 0 && p <= sample_prob[k] && p > sample_prob[k - 1]) {
        //         _y_vec(y_idx, k) += 1;
        //       }
        //     }
        //   }

          MatrixYType _ybar;
          _ybar = (Ks * _y_vec);

          for (int r = 0; r < _ybar.rows(); ++r) {
            for (int k = 0; k < _ybar.cols(); ++k) {
              ybars[r][k] = _ybar(r, k);
            }
          }
        // }
        // }
      }

        
    private:
        /*
         * @brief Compute Euclid distances between two vectors.
         * @param x input vector
         * @param z input vecotr
         * @return d distance matrix
         */
        void dist(const MatrixXType &x, const MatrixXType &z, MatrixKType &d) const {
            d = MatrixKType::Zero(x.rows(), z.rows());
            for (int i = 0; i < x.rows(); ++i) {
                d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();
            }
        }

        /*
         * @brief Matern3 kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         */
        void covMaterniso3(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(1.73205 / ell * x, 1.73205 / ell * z, Kxz);
            Kxz = ((1 + Kxz.array()) * exp(-Kxz.array())).matrix() * sf2;
        }

        /*
         * @brief Sparse kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         * @ref A sparse covariance function for exact gaussian process inference in large datasets.
         */
        void covSparse(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(x / ell, z / ell, Kxz);
            Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
                  (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * sf2;

            // Clean up for values with distance outside length scale
            // Possible because Kxz <= 0 when dist >= ell
            for (int i = 0; i < Kxz.rows(); ++i)
            {
                for (int j = 0; j < Kxz.cols(); ++j)
                    if (Kxz(i,j) < 0.0)
                        Kxz(i,j) = 0.0f;
            }
        }

        void covCountingSensorModel(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
          Kxz = MatrixKType::Ones(x.rows(), z.rows());
        }

        T sf2;    // signal variance
        T ell;    // length-scale
        int nc;   // number of classes

        MatrixXType x;   // temporary storage of training data
        MatrixYType y;   // temporary storage of training labels
        std::vector<std::vector<T> > y_vec;

        bool trained;    // true if bgkinference stored training data
    };

    typedef SemanticBKInference<3, float> SemanticBKI3f;

}
