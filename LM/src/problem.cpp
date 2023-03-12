//
// Created by Cain on 2023/2/21.
//

#include <problem.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
//#include <glog/logging.h>

using namespace std;

namespace vio {
    Problem::Problem(ProblemType problem_type) :
            _problem_type(problem_type) {
        logout_vector_size();
        _vertices_marg.clear();
    }

    void Problem::logout_vector_size() {
        // LOG(INFO) <<
        //           "1 problem::LogoutVectorSize verticies_:" << verticies_.size() <<
        //           " edges:" << edges_.size();
    }
}