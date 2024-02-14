#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ur_pers_lib_pkg/3DTransformations/MatrixVectorRelations.hpp>


class ListOfEndEffectorRotation{
    
    public:
        ListOfEndEffectorRotation();
        ~ListOfEndEffectorRotation();
        void removeNonComplanarRotations();
        Eigen::Matrix3d getMatrix(int i) const;
        Eigen::VectorXd getAngles(int i) const;
        int size() const;
        std::vector<Eigen::Matrix3d> getList() const;
        void print() const;

        Eigen::Matrix3d operator[](int i) const;
        Eigen::VectorXd operator()(int i) const;

    private:
        std::vector<Eigen::Matrix3d> eeRot;


};
