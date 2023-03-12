//
// Created by Cain on 2023/2/20.
//

#ifndef LM_VERTEX_H
#define LM_VERTEX_H

#include <eigen_types.h>

namespace vio {
    class Vertex {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    public:
        /*!
         * 构造函数
         * @param num_dimension - 顶点自身维度
         * @param local_dimension - 本地参数化维度, 为-1时认为与本身维度一样
         */
        explicit Vertex(int num_dimension, int local_dimension = -1);

        virtual ~Vertex() = default;;

        /// 返回变量维度
        int dimension() const { return  _parameters.rows(); };

        /// 返回变量本地维度
        int local_dimension() const { return _local_dimension; };

        /// 该顶点的id
        unsigned long id() const { return _id; }

        /// 返回参数值
        VecX parameters() const { return _parameters; }

        /// 返回参数值的引用
        VecX &parameters() { return _parameters; }

        /// 设置参数值
        void set_parameters(const VecX &params) { _parameters = params; }

        /// 加法，可重定义
        /// 默认是向量加
        virtual void plus(const VecX &delta) = 0;

        /// 返回顶点的名称，在子类中实现
        virtual std::string type_info() const = 0;

        unsigned long ordering_id() const { return _ordering_id; }

        void set_ordering_id(unsigned long id) { _ordering_id = id; };

        /// 固定该点的估计值
        void set_fixed(bool fixed = true) {
            _fixed = fixed;
        }

        /// 测试该点是否被固定
        bool is_fixed() const { return _fixed; }

    protected:
        VecX _parameters;   ///< 实际存储的变量值
        int _local_dimension;   ///< 局部参数化维度
        unsigned long _id;  ///< 顶点的id，自动生成

        /// ordering id是在problem中排序后的id，用于寻找雅可比对应块
        /// ordering id带有维度信息，例如ordering_id=6则对应Hessian中的第6列
        /// 从零开始
        unsigned long _ordering_id = 0;

        bool _fixed = false;    ///< 是否固定

    private:
        static unsigned long _global_vertex_id;
    };
}

#endif //LM_VERTEX_H
