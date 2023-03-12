//
// Created by Cain on 2023/2/20.
//

#ifndef LM_EDGE_H
#define LM_EDGE_H

#include <eigen_types.h>
#include <vertex.h>
#include <memory>
#include <string>

namespace vio {
    class Edge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        explicit Edge(int residual_dimension, int num_vertices,
                      const std::vector<std::string> &vertices_types = std::vector<std::string>());

        virtual ~Edge() = default;

        /*!
         * 获取id
         * @return _id
         */
        unsigned long id() const { return _id; }

        /*!
         * 新增一个顶点
         * @param vertex - 对应的vertex对象
         * @return
         */
        bool add_vertex(const std::shared_ptr<Vertex>& vertex) {
            _vertices.emplace_back(vertex);
            return true;
        }

        /*!
         * 设置所有顶点
         * @param vertices - 顶点，按引用顺序排列
         * @return
         */
        bool set_vertex(const std::vector<std::shared_ptr<Vertex>> &vertices) {
            _vertices = vertices;
            return true;
        }

        /*!
         * 返回第i个顶点
         * @param i - 顶点对应的index
         * @return 相应的顶点
         */
        std::shared_ptr<Vertex> get_vertex(int i) const {
            return _vertices[i];
        }

        /*!
         * 返回所有顶点
         * @return 所有顶点
         */
        std::vector<std::shared_ptr<Vertex>> vertices() const {
            return _vertices;
        }

        /*!
         * 返回顶点个数
         * @return 顶点个数
         */
        size_t num_vertices() const { return _vertices.size(); }

        /*!
         * 返回边的类型信息，在子类中实现
         * @return 边的类型信息
         */
        virtual std::string type_info() const = 0;

        /*!
         * 计算残差，由子类实现
         */
        virtual void compute_residual() = 0;

        /*!
         * 计算雅可比，由子类实现
         * 本后端不支持自动求导，需要实现每个子类的雅可比计算方法
         */
        virtual void compute_jacobians() = 0;

        /*!
         * 计算平方误差，会乘以信息矩阵
         * @return 平方误差
         */
        double chi2();

        /*!
         * 返回残差
         * @return 残差
         */
        VecX residual() const { return _residual; }

        /*!
         * 返回雅可比
         * @return 雅可比
         */
        std::vector<MatXX> jacobians() const { return _jacobians; }

        /*!
         * 返回信息矩阵
         * @return 信息矩阵
         */
        MatXX information() const { return _information; }

        /*!
         * 返回观测信息
         * @return 观测信息
         */
        VecX observation() const { return _observation; }

        /*!
         * 返回ordering_id
         * @return ordering_id
         */
        int ordering_id() const { return _ordering_id; }

        /*!
         * 设置信息矩阵, information_ = sqrt_Omega = w
         * @param information - 信息矩阵
         */
        void set_information(const MatXX &information) { _information = information; }

        /*!
         * 设置观测信息
         * @param observation - 观测信息
         */
        void set_observation(const VecX &observation) { _observation = observation; }

        /*!
         * 设置ordering_id
         * @param id - ordering_id
         */
        void set_ordering_id(int id) { _ordering_id = id; };

        /*!
         * 检查边的信息是否全部设置
         * @return 信息是否全部设置
         */
        bool check_valid();

    protected:
        unsigned long _id;  ///< edge id
        int _ordering_id;   ///< edge id in problem
        std::vector<std::string> _vertices_types;  ///< 各顶点类型信息，用于debug
        std::vector<std::shared_ptr<Vertex>> _vertices; ///< 该边对应的顶点
        VecX _residual;                 ///< 残差
        std::vector<MatXX> _jacobians;  ///< 雅可比，每个雅可比维度是 residual x vertex[i]
        MatXX _information;             ///< 信息矩阵
        VecX _observation;              ///< 观测信息

    private:
        static unsigned long _global_edge_id;
    };
}

#endif //LM_EDGE_H
