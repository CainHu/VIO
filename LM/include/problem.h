//
// Created by Cain on 2023/2/21.
//

#ifndef LM_PROBLEM_H
#define LM_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include <eigen_types.h>
#include <vertex.h>
#include <edge.h>

namespace vio {
    class Problem {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    public:
        /**
         * 问题的类型
         * SLAM问题还是通用的问题
         *
         * 如果是SLAM问题那么pose和landmark是区分开的，Hessian以稀疏方式存储
         * SLAM问题只接受一些特定的Vertex和Edge
         * 如果是通用问题那么hessian是稠密的，除非用户设定某些vertex为marginalized
         */
        enum class ProblemType {
            SLAM_PROBLEM,
            GENERIC_PROBLEM
        };

        typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
        typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
        typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

        Problem(ProblemType problem_type);

        ~Problem() = default;

        /*!
         * 增加一个顶点
         * @param vertex - 所新增的顶点
         * @return
         */
        bool add_vertex(std::shared_ptr<Vertex> vertex);

        /*!
         * 移除一个顶点
         * @param vertex - 所要移除的顶点
         * @return
         */
        bool remove_vertex(std::shared_ptr<Vertex> vertex);

        /*!
         * 增加一条边
         * @param edge - 所增加的边
         * @return
         */
        bool add_edge(std::shared_ptr<Edge> edge);

        /*!
         * 移除一条边
         * @param edge - 所移除的边
         * @return
         */
        bool remove_edge(std::shared_ptr<Edge> edge);

        /*!
         * 取得在优化中被判断为outlier部分的边，方便前端去除outlier
         * @param outlier_edges - 被判断为outlier部分的边
         */
        void get_outlier_edges(std::vector<std::shared_ptr<Edge>> &outlier_edges);

        /*!
         * 求解此问题
         * @param iterations - 迭代步数
         * @return
         */
        bool solve(int iterations);

        /*!
         * 边缘化一个frame和以它为host的landmark
         * @param frame_vertex
         * @param landmark_verticies
         * @return
         */
        bool marginalize(std::shared_ptr<Vertex> frame_vertex,
                         const std::vector<std::shared_ptr<Vertex>> &landmark_verticies);

        /*!
         *
         * @param frameVertex
         * @return
         */
        bool marginalize(const std::shared_ptr<Vertex> frameVertex);

        //test compute prior
        void test_compute_prior();

    protected:
        /*!
         * Solve的实现，解通用问题
         * @param iterations - 迭代次数
         * @return
         */
        bool solve_generic_problem(int iterations);

        /*!
         * Solve的实现，解SLAM问题
         * @param iterations - 迭代次数
         * @return
         */
        bool solve_slam_problem(int iterations);

        /*!
         * 设置各顶点的ordering_index
         */
        void set_ordering();

        /*!
         * set ordering for new vertex in slam problem
         * @param v
         */
        void add_ordering_slam(std::shared_ptr<Vertex> v);

        /*!
         * 构造Hessian矩阵
         */
        void make_hessian();

        /*!
         * schur求解SBA
         */
        void schur_sba();

        /*!
         * 解线性方程
         */
        void solve_linear_system();

        /*!
         * 更新状态变量
         */
        void update_states();

        /*!
         * 有时候 update 后残差会变大，需要退回去，重来
         */
        void rollback_states();

        /*!
         * 计算并更新Prior部分
         */
        void compute_prior();

        /*!
         * 判断一个顶点是否为Pose顶点
         * @param v
         * @return
         */
        bool is_pose_vertex(std::shared_ptr<Vertex> v);

        /*!
         * 判断一个顶点是否为landmark顶点
         * @param v
         * @return
         */
        bool is_landmark_vertex(std::shared_ptr<Vertex> v);

        /*!
         * 在新增顶点后，需要调整几个hessian的大小
         * @param v
         */
        void resize_pose_hessians_when_adding_pose(std::shared_ptr<Vertex> v);

        /*!
         * 检查ordering是否正确
         * @return
         */
        bool check_ordering();

        /*!
         * 记录
         */
        void logout_vector_size();

        /*!
         * 获取某个顶点连接到的边
         * @param vertex
         * @return
         */
        std::vector<std::shared_ptr<Edge>> get_connected_edges(std::shared_ptr<Vertex> vertex);

        /*!
         * Levenberg, 计算LM算法的初始Lambda
         */
        void compute_lambda_init_lm();

        /*!
         * Hessian 对角线加上 Lambda
         */
        void add_lambda_to_hessian_lm();

        /*!
         * Hessian 对角线减去 Lambda
         */
        void remove_lambda_hessian_lm();

        /*!
         * LM 算法中用于判断 Lambda 在上次迭代中是否可以，以及Lambda怎么缩放
         * @return
         */
        bool is_good_step_in_lm();

        /*!
         * PCG 迭代线性求解器
         * @param A
         * @param b
         * @param maxIter
         * @return
         */
        VecX pcg_solver(const MatXX &A, const VecX &b, int maxIter);

        double _current_lambda;
        double _current_chi;
        double _stop_threshold_lm;    // LM 迭代退出阈值条件
        double _ni;                 //控制 Lambda 缩放大小

        ProblemType _problem_type;

        /// 整个信息矩阵
        MatXX _Hessian;
        VecX _b;
        VecX _delta_x;

        /// 先验部分信息
        MatXX _H_prior;
        VecX _b_prior;
        MatXX _Jt_prior_inv;
        VecX _err_prior;

        /// SBA的Pose部分
        MatXX _H_pp_schur;
        VecX _b_pp_schur;

        /// Heesian 的 Landmark 和 pose 部分
        MatXX _H_pp;
        VecX _b_pp;
        MatXX _H_ll;
        VecX _b_ll;

        /// all vertices
        HashVertex _vertices;

        /// all edges
        HashEdge _edges;

        /// 由vertex id查询edge
        HashVertexIdToEdge _vertex_to_edge;

        /// Ordering related
        unsigned long _ordering_poses = 0;
        unsigned long _ordering_landmarks = 0;
        unsigned long _ordering_generic = 0;
        std::map<unsigned long, std::shared_ptr<Vertex>> _idx_pose_vertices;        // 以ordering排序的pose顶点
        std::map<unsigned long, std::shared_ptr<Vertex>> _idx_landmark_vertices;    // 以ordering排序的landmark顶点

        // vertices need to marg. <Ordering_id_, Vertex>
        HashVertex _vertices_marg;

        bool _bDebug = false;
        double _t_hessian_cost = 0.0;
        double _t_pcg_sovle_cost = 0.0;

    private:
    };
}

#endif //LM_PROBLEM_H
