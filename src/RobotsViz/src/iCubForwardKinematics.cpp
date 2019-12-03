/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/iCubForwardKinematics.h>

using namespace Eigen;
using namespace RobotsViz;
using Tr = Translation<double, 3>;
using R = AngleAxisd;


iCubForwardKinematics::iCubForwardKinematics(const std::string& part_name)
{
    if (part_name == "left_hand")
    {
    /* Map from end-effector to top cover. */
        maps_["ee"]["top_cover"] = T::Identity();


    /* Map from end-effector to palm. */

        /* Virtual root to end-effector. */
        T vr_ee = DH(0.016, 0.0, 0.0625, M_PI);

        /* Virtual root to palm. */
        T vr_palm = DH(0.012, 0.0, 0.0, 0.0);

        /* end-effector to palm. */
        T ee_palm = vr_ee.inverse() * vr_palm;

        maps_["ee"]["palm"] = ee_palm;

    /* ***************************** */

    /* Map from end-effector to thumb. */

        /* Palm to index0. */
        T palm_thumb0;
        palm_thumb0 = Tr(Vector3d(0.039, 0.012, 0.0025));
        palm_thumb0.rotate(R(4.76364 * M_PI / 180.0, Vector3d::UnitY()) * R(-29.04 * M_PI / 180.0, Vector3d::UnitZ()));
        palm_thumb0.rotate(R(M_PI / 2.0, Vector3d::UnitY()) * R(M_PI / 2.0, Vector3d::UnitZ()));

        /* Virtual root to thumb0. */
        T vr_thumb0 = vr_palm * palm_thumb0;

        /* end-effector to thumb0. */
        T ee_thumb0 = vr_ee.inverse() * vr_thumb0;

        maps_["ee"]["thumb0"] = ee_thumb0;

        /* thumb0 to thumb0_r. */
        T thumb0_r = T::Identity();
        thumb0_r.rotate(R(M_PI, Vector3d::UnitX()));
        maps_["thumb0"]["thumb0_r"] = thumb0_r;

        /* thumb0 to thumb1. */
        T thumb0_thumb1 = DH(0.0, 0.0, 0.0211, M_PI / 2.0) * DH(-0.0049, 0.0, 0.0, 0.0);
        maps_["thumb0"]["thumb1"] = thumb0_thumb1;

        /* thumb1 to thumb1_r. */
        T thumb1_thumb1_r = T::Identity();
        thumb1_thumb1_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["thumb1"]["thumb1_r"] = thumb1_thumb1_r;

        /* thumb1 to thumb2. */
        T thumb1_thumb2 = DH(0.0, 0.0, 0.026, 0.0);
        maps_["thumb1"]["thumb2"] = thumb1_thumb2;

        /* thumb2 to  thumb2_r. */
        T thumb2_thumb2_r = T::Identity();
        thumb2_thumb2_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["thumb2"]["thumb2_r"] = thumb2_thumb2_r;

        /* thumb2 to thumb3. */
        T thumb2_thumb3 = DH(0.0, 0.0, 0.022, 0.0);
        maps_["thumb2"]["thumb3"] = thumb2_thumb3;

        /* thumb3 to  thumb3_r. */
        T thumb3_thumb3_r = T::Identity();
        thumb3_thumb3_r.rotate(R(M_PI, Vector3d::UnitY()));
        maps_["thumb3"]["thumb3_r"] = thumb3_thumb3_r;

    /* ***************************** */

    /* Map from end-effector to index. */

        /* Palm to index0. */
        T palm_index0;
        palm_index0 = Tr(Vector3d(0.06425, 0.02622, -0.0067));
        palm_index0.rotate(R(5.0 * M_PI / 180.0, Vector3d::UnitX()) * R(4.1 * M_PI / 180.0, Vector3d::UnitZ()));

        /* Virtual root to index0. */
        T vr_index0 = vr_palm * palm_index0;

        /* end-effector to index0. */
        T ee_index0 = vr_ee.inverse() * vr_index0;

        maps_["ee"]["index0"] = ee_index0;

        /* index0 to index1. */
        T index0_index1 = DH(0.0, 0.0, 0.0148, M_PI / 2.0) * DH(0.00175, 0.0, 0.0, 0.0);
        maps_["index0"]["index1"] = index0_index1;

        /* index1 to index1_r. */
        T index1_index1_r = T::Identity();
        index1_index1_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["index1"]["index1_r"] = index1_index1_r;

        /* index1 to index2. */
        T index1_index2 = DH(0.0, 0.0, 0.0259, 0.0);
        maps_["index1"]["index2"] = index1_index2;

        T index2_index2_r = T::Identity();
        index2_index2_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["index2"]["index2_r"] = index2_index2_r;

        /* index2 to index3. */
        T index2_index3 = DH(0.0, 0.0, 0.022, 0.0);
        maps_["index2"]["index3"] = index2_index3;

        T index3_index3_r = T::Identity();
        index3_index3_r.rotate(R(M_PI, Vector3d::UnitY()));
        maps_["index3"]["index3_r"] = index3_index3_r;

    /* ***************************** */

    /* Map from end-effector to middle. */

        /* Palm to middle0. */
        T palm_middle0;
        palm_middle0 = Tr(Vector3d(0.065, 0.00977, -0.0076));

        /* Virtual root to middle0. */
        T vr_middle0 = vr_palm * palm_middle0;

        /* end-effector to middle0. */
        T ee_middle0 = vr_ee.inverse() * vr_middle0;

        maps_["ee"]["middle0"] = ee_middle0;

        /* middle0 to middle0_r. */
        T middle0_middle0_r = T::Identity();
        middle0_middle0_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI / 2.0, Vector3d::UnitZ()));
        maps_["middle0"]["middle0_r"] = middle0_middle0_r;

        /* middle0 to middle1. */
        T middle0_middle1 = DH(0.0, 0.0, 0.0153, M_PI / 2.0) * DH(0.0022, 0.0, 0.0, 0.0);
        maps_["middle0"]["middle1"] = middle0_middle1;

        /* middle1 to middle1_r. */
        T middle1_middle1_r = T::Identity();
        middle1_middle1_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["middle1"]["middle1_r"] = middle1_middle1_r;

        /* middle1 to middle2. */
        T middle1_middle2 = DH(0.0, 0.0, 0.0258, 0.0);
        maps_["middle1"]["middle2"] = middle1_middle2;

        T middle2_middle2_r = T::Identity();
        middle2_middle2_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["middle2"]["middle2_r"] = middle2_middle2_r;

        /* middle2 to middle3. */
        T middle2_middle3 = DH(0.0, 0.0, 0.024, 0.0);
        maps_["middle2"]["middle3"] = middle2_middle3;

        T middle3_middle3_r = T::Identity();
        middle3_middle3_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["middle3"]["middle3_r"] = middle3_middle3_r;

    /* ***************************** */

    /* Map from end-effector to ring. */

        /* Palm to ring0. */
        T palm_ring0;
        palm_ring0 = Tr(Vector3d(0.06413, -0.00565, -0.007));
        palm_ring0.rotate(R(-5.0 * M_PI / 180.0, Vector3d::UnitY()) * R(-6.1 * M_PI / 180.0, Vector3d::UnitZ()));

        /* Virtual root to ring0. */
        T vr_ring0 = vr_palm * palm_ring0;

        /* end-effector to ring0. */
        T ee_ring0 = vr_ee.inverse() * vr_ring0;

        maps_["ee"]["ring0"] = ee_ring0;

        /* ring0 to ring1. */
        T ring0_ring1 = DH(0.0, 0.0, 0.0148, M_PI / 2.0) * DH(0.00175, 0.0, 0.0, 0.0);
        maps_["ring0"]["ring1"] = ring0_ring1;

        /* ring1 to ring1_r. */
        T ring1_ring1_r = T::Identity();
        ring1_ring1_r.rotate(R(M_PI, Vector3d::UnitZ()) * R(M_PI / 2.0, Vector3d::UnitX()));
        maps_["ring1"]["ring1_r"] = ring1_ring1_r;

        /* ring1 to ring2. */
        T ring1_ring2 = DH(0.0, 0.0, 0.0259, 0.0);
        maps_["ring1"]["ring2"] = ring1_ring2;

        T ring2_ring2_r = T::Identity();
        ring2_ring2_r.rotate(R(M_PI, Vector3d::UnitY()) * R(M_PI, Vector3d::UnitX()));
        maps_["ring2"]["ring2_r"] = ring2_ring2_r;

        /* ring2 to ring3. */
        T ring2_ring3 = DH(0.0, 0.0, 0.022, 0.0);
        maps_["ring2"]["ring3"] = ring2_ring3;

        T ring3_ring3_r = T::Identity();
        ring3_ring3_r.rotate(R(M_PI, Vector3d::UnitY()));
        maps_["ring3"]["ring3_r"] = ring3_ring3_r;

    /* ***************************** */

    /* Map from end-effector to little. */

        /* Palm to ring0. */
        T palm_little0;
        palm_little0 = Tr(Vector3d(0.06211, -0.02454, -0.006));
        palm_little0.rotate(R(-5.0 * M_PI / 180.0, Vector3d::UnitY()) * R(-6.1 * M_PI / 180.0, Vector3d::UnitZ()));

        /* Virtual root to little0. */
        T vr_little0 = vr_palm * palm_little0;

        /* end-effector to little0. */
        T ee_little0 = vr_ee.inverse() * vr_little0;

        maps_["ee"]["little0"] = ee_little0;

        /* little0 to little0_r. */
        T little0_little0_r = T::Identity();
        little0_little0_r.rotate(R(-1.0 * M_PI / 2.0, Vector3d::UnitZ()) * R(-1.0 * M_PI / 2.0, Vector3d::UnitY()));
        maps_["little0"]["little0_r"] = little0_little0_r;

        /* little0 to little1. */
        T little0_little1 = DH(0.0, 0.0, 0.0148, M_PI / 2.0) * DH(-0.00175, 0.0, 0.0, 0.0);
        maps_["little0"]["little1"] = little0_little1;

        /* little1 to little1_r. */
        T little1_little1_r = T::Identity();
        little1_little1_r.rotate(R(M_PI / 2.0, Vector3d::UnitZ()) * R(M_PI, Vector3d::UnitY()));
        maps_["little1"]["little1_r"] = little1_little1_r;

        /* little1 to little2. */
        T little1_little2 = DH(0.0, 0.0, 0.0219, 0.0);
        maps_["little1"]["little2"] = little1_little2;

        T little2_little2_r = T::Identity();
        little2_little2_r.rotate(R(M_PI, Vector3d::UnitZ()));
        maps_["little2"]["little2_r"] = little2_little2_r;

        /* little2 to little3. */
        T little2_little3 = DH(0.0, 0.0, 0.019, 0.0);
        maps_["little2"]["little3"] = little2_little3;

        T little3_little3_r = T::Identity();
        little3_little3_r.rotate(R(M_PI, Vector3d::UnitY()));
        maps_["little3"]["little3_r"] = little3_little3_r;

    /* ***************************** */

    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Cannot find part " + part_name + "."));
}


iCubForwardKinematics::~iCubForwardKinematics()
{}


T iCubForwardKinematics::map(const std::string& from, const std::string& to, const std::unordered_map<std::string, Eigen::VectorXd>& encoders)
{
    T map;

    if (from == "ee" && to == "palm")
        map =  maps_.at("ee").at("palm");
    else if (from == "ee")
    {
        if (to.find("thumb") != to.npos)
        {
            T thumb_0 = maps_.at("ee").at("thumb0") * R(encoders.at("thumb")(0), Vector3d::UnitZ());
            T thumb_1 = thumb_0 * maps_.at("thumb0").at("thumb1") * R(encoders.at("thumb")(1), Vector3d::UnitZ());
            T thumb_2 = thumb_1 * maps_.at("thumb1").at("thumb2") * R(encoders.at("thumb")(2), Vector3d::UnitZ());
            T thumb_3 = thumb_2 * maps_.at("thumb2").at("thumb3") * R(encoders.at("thumb")(3), Vector3d::UnitZ());

            if (to == "thumb0")
                map = thumb_0 * maps_.at("thumb0").at("thumb0_r");
            else if (to == "thumb1")
                map = thumb_1 * maps_.at("thumb1").at("thumb1_r");
            else if (to == "thumb2")
                map = thumb_2 * maps_.at("thumb2").at("thumb2_r");
            else if (to == "thumb3")
                map = thumb_3 * maps_.at("thumb3").at("thumb3_r");
        }
        else if (to.find("index") != to.npos)
        {
            T index_0 = maps_.at("ee").at("index0") * R(encoders.at("index")(0), Vector3d::UnitZ());
            T index_1 = index_0 * maps_.at("index0").at("index1") * R(encoders.at("index")(1), Vector3d::UnitZ());
            T index_2 = index_1 * maps_.at("index1").at("index2") * R(encoders.at("index")(2), Vector3d::UnitZ());
            T index_3 = index_2 * maps_.at("index2").at("index3") * R(encoders.at("index")(3), Vector3d::UnitZ());

            if (to == "index0")
                map = index_0;
            else if (to == "index1")
                map = index_1 * maps_.at("index1").at("index1_r");
            else if (to == "index2")
                map = index_2 * maps_.at("index2").at("index2_r");
            else if (to == "index3")
                map = index_3 * maps_.at("index3").at("index3_r");
        }
        else if (to.find("middle") != to.npos)
        {
            T middle_0 = maps_.at("ee").at("middle0");
            T middle_1 = middle_0 * maps_.at("middle0").at("middle1") * R(encoders.at("middle")(0), Vector3d::UnitZ());
            T middle_2 = middle_1 * maps_.at("middle1").at("middle2") * R(encoders.at("middle")(1), Vector3d::UnitZ());
            T middle_3 = middle_2 * maps_.at("middle2").at("middle3") * R(encoders.at("middle")(2), Vector3d::UnitZ());

            if (to == "middle0")
                map = middle_0 * maps_.at("middle0").at("middle0_r");
            else if (to == "middle1")
                map = middle_1 * maps_.at("middle1").at("middle1_r");
            else if (to == "middle2")
                map = middle_2 * maps_.at("middle2").at("middle2_r");
            else if (to == "middle3")
                map = middle_3 * maps_.at("middle3").at("middle3_r");
        }
        else if (to.find("ring") != to.npos)
        {
            T ring_0 = maps_.at("ee").at("ring0") * R(-1 * encoders.at("ring")(0), Vector3d::UnitZ());
            T ring_1 = ring_0 * maps_.at("ring0").at("ring1") * R(encoders.at("ring")(1), Vector3d::UnitZ());
            T ring_2 = ring_1 * maps_.at("ring1").at("ring2") * R(encoders.at("ring")(2), Vector3d::UnitZ());
            T ring_3 = ring_2 * maps_.at("ring2").at("ring3") * R(encoders.at("ring")(3), Vector3d::UnitZ());

            if (to == "ring0")
                map = ring_0;
            else if (to == "ring1")
                map = ring_1 * maps_.at("ring1").at("ring1_r");
            else if (to == "ring2")
                map = ring_2 * maps_.at("ring2").at("ring2_r");
            else if (to == "ring3")
                map = ring_3 * maps_.at("ring3").at("ring3_r");
        }
        else if (to.find("little") != to.npos)
        {
            T little_0 = maps_.at("ee").at("little0") * R(-1 * encoders.at("little")(0), Vector3d::UnitZ());
            T little_1 = little_0 * maps_.at("little0").at("little1") * R(encoders.at("little")(1), Vector3d::UnitZ());
            T little_2 = little_1 * maps_.at("little1").at("little2") * R(encoders.at("little")(2), Vector3d::UnitZ());
            T little_3 = little_2 * maps_.at("little2").at("little3") * R(encoders.at("little")(3), Vector3d::UnitZ());

            if (to == "little0")
                map = little_0 * maps_.at("little0").at("little0_r");
            else if (to == "little1")
                map = little_1 * maps_.at("little1").at("little1_r");
            else if (to == "little2")
                map = little_2 * maps_.at("little2").at("little2_r");
            else if (to == "little3")
                map = little_3 * maps_.at("little3").at("little3_r");
        }
    }

    return map;
}


T iCubForwardKinematics::map(const std::string& from, const std::string& to)
{
    std::unordered_map<std::string, Eigen::VectorXd> zeroed_encoders;

    /* Assume zeroed configuration for fingers. */
    VectorXd zero_4 = VectorXd::Zero(4);
    VectorXd zero_middle = VectorXd::Zero(3);

    zeroed_encoders["thumb"] = zero_4;
    zeroed_encoders["index"] = zero_4;
    zeroed_encoders["middle"] = zero_middle;
    zeroed_encoders["ring"] = zero_4;
    zeroed_encoders["little"] = zero_4;

    return map(from, to, zeroed_encoders);
}


T iCubForwardKinematics::DH(const double& d, const double& theta, const double& a, const double& alpha)
{
    T dh;

    dh = Tr(Vector3d(0.0, 0.0, d)) *
         R(theta, Vector3d::UnitZ()) *
         Tr(Vector3d(a, 0.0, 0.0)) *
         R(alpha, Vector3d::UnitX());

    return dh;
}
