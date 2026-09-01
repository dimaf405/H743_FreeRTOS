#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2023 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: derivation.py
Description:
    Derivation of an error-state EKF based on
    Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).
    The derivation is directly done in discrete-time as this allows us to define the desired type of discretization
    for each element while defining the equations (easier than a continuous-time derivation followed by a block-wise discretization).
"""

import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce import typing as T
from symforce import ops
from symforce.values import Values

import sympy as sp
from utils.derivation_utils import *

# The state vector is organized in an ordered dictionary
# Dima N1 的状态闭包固定包含 IMU bias 与地磁状态，不包含 Wind 和 Terrain。
# 这里直接定义唯一产品模型，避免通过命令行生成多套互不一致的状态和协方差公式。
State = Values(
    quat_nominal = sf.Rot3(),
    vel = sf.V3(),
    pos = sf.V3(),
    gyro_bias = sf.V3(),
    accel_bias = sf.V3(),
    mag_I = sf.V3(),
    mag_B = sf.V3()
)

class IdxDof():
    def __init__(self, idx, dof):
        self.idx = idx
        self.dof = dof

def BuildTangentStateIndex():
    # Build a dictionary that can be used to access elements of vectors
    # and matrices defined in the state tangent space (e.g.: P, K and H)
    tangent_state_index = {}
    idx = 0
    for key in State.keys_recursive():
        dof = State[key].tangent_dim()
        tangent_state_index[key] = IdxDof(idx, dof)
        idx += dof
    return tangent_state_index

tangent_idx = BuildTangentStateIndex()

class VState(sf.Matrix):
    SHAPE = (State.storage_dim(), 1)

class VTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), 1)

class MTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), State.tangent_dim())

def vstate_to_state(v: VState):
    state = State.from_storage(v)
    q_px4 = state["quat_nominal"].to_storage()
    state["quat_nominal"] = sf.Rot3(sf.Quaternion(xyz=sf.V3(q_px4[1], q_px4[2], q_px4[3]), w=q_px4[0]))
    return state

def predict_covariance(
    state: VState,
    P: MTangent,
    accel: sf.V3,
    accel_var: sf.V3,
    gyro: sf.V3,
    gyro_var: sf.Scalar,
    dt: sf.Scalar
) -> MTangent:

    state = vstate_to_state(state)
    g = sf.Symbol("g") # does not appear in the jacobians

    state_error = Values(
        theta = sf.V3.symbolic("delta_theta"),
        vel = sf.V3.symbolic("delta_v"),
        pos = sf.V3.symbolic("delta_p"),
        gyro_bias = sf.V3.symbolic("delta_w_b"),
        accel_bias = sf.V3.symbolic("delta_a_b"),
        mag_I = sf.V3.symbolic("mag_I"),
        mag_B = sf.V3.symbolic("mag_B")
    )

    # True state kinematics
    state_t = Values()

    for key in state.keys():
        if key == "quat_nominal":
            # Create true quaternion using small angle approximation of the error rotation
            state_t["quat_nominal"] = sf.Rot3(sf.Quaternion(xyz=(state_error["theta"] / 2), w=1)) * state["quat_nominal"]
        else:
            state_t[key] = state[key] + state_error[key]

    noise = Values(
        accel = sf.V3.symbolic("a_n"),
        gyro = sf.V3.symbolic("w_n"),
    )

    input_t = Values(
        accel = accel - state_t["accel_bias"] - noise["accel"],
        gyro = gyro - state_t["gyro_bias"] - noise["gyro"]
    )

    R_t = state_t["quat_nominal"]
    state_t_pred = state_t.copy()
    state_t_pred["quat_nominal"] = state_t["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input_t["gyro"] * dt / 2), w=1))
    state_t_pred["vel"] = state_t["vel"] + (R_t * input_t["accel"] + sf.V3(0, 0, g)) * dt
    state_t_pred["pos"] = state_t["pos"] + state_t["vel"] * dt

    # Nominal state kinematics
    input = Values(
        accel = accel - state["accel_bias"],
        gyro = gyro - state["gyro_bias"]
    )

    R = state["quat_nominal"]
    state_pred = state.copy()
    state_pred["quat_nominal"] = state["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input["gyro"] * dt / 2), w=1))
    state_pred["vel"] = state["vel"] + (R * input["accel"] + sf.V3(0, 0, g)) * dt
    state_pred["pos"] = state["pos"] + state["vel"] * dt

    # Error state kinematics
    state_error_pred = Values()
    for key in state_error.keys():
        if key == "theta":
            delta_q = sf.Quaternion.from_storage(state_t_pred["quat_nominal"].to_storage()) * sf.Quaternion.from_storage(state_pred["quat_nominal"].to_storage()).conj()
            state_error_pred["theta"] = 2 * sf.V3(delta_q.x, delta_q.y, delta_q.z) # Use small angle approximation to obtain a simpler jacobian
        else:
            state_error_pred[key] = state_t_pred[key] - state_pred[key]

    # Simplify angular error state prediction
    for i in range(state_error_pred["theta"].storage_dim()):
        state_error_pred["theta"][i] = sp.expand(state_error_pred["theta"][i]).subs(dt**2, 0) # do not consider dt**2 effects in the derivation
        q_est = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
        state_error_pred["theta"][i] = sp.factor(state_error_pred["theta"][i]).subs(q_est.w**2 + q_est.x**2 + q_est.y**2 + q_est.z**2, 1) # unit norm quaternion

    zero_state_error = {state_error[key]: state_error[key].zero() for key in state_error.keys()}
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}

    # State propagation jacobian
    A = VTangent(state_error_pred.to_storage()).jacobian(state_error).subs(zero_state_error).subs(zero_noise)
    G = VTangent(state_error_pred.to_storage()).jacobian(noise).subs(zero_state_error).subs(zero_noise)

    # Covariance propagation
    var_u = sf.Matrix.diag([accel_var[0], accel_var[1], accel_var[2], gyro_var, gyro_var, gyro_var])
    P_new = A * P * A.T + G * var_u * G.T

    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(state.tangent_dim()):
        for j in range(state.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return P_new

def jacobian_chain_rule(expr: sf.Scalar , state: State):
    # First compute the jacobian in the parameter space
    dh_dx = sf.V1(expr).jacobian(state, tangent_space=False)

    class MStorageTangent(sf.Matrix):
        SHAPE = (State.storage_dim(), State.tangent_dim())

    # Then compute the jarobian mapping infinitesimal elements of the parameter space to the error state
    # Note that this jacobian only depends on the structure of the EKF
    dx_derror = MStorageTangent()
    q = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
    p = sf.Quaternion.symbolic('p')

    pq = p * q
    qR = sf.M41(pq.to_storage()).jacobian(sf.M41(p.to_storage())) # Right quaternion product matrix
    dx_derror[0:4, 0:3] = qR / 2 * sf.M43([[1, 0, 0],
                                           [0, 1, 0],
                                           [0, 0, 1],
                                           [0, 0, 0]])

    # The rest of the matrix is trivial
    for i in range(4, State.storage_dim()):
        for j in range(3, State.tangent_dim()):
            if (i == j+1):
                dx_derror[i, j] = 1

    # Finally use the chain rule: dh/derror = dh/dx * dx/derror
    H = dh_dx * dx_derror
    return H

def predict_mag_body(state) -> sf.V3:
    mag_field_earth = state["mag_I"]
    mag_bias_body = state["mag_B"]

    mag_body = state["quat_nominal"].inverse() * mag_field_earth + mag_bias_body
    return mag_body

def compute_mag_innov_innov_var_and_hx(
        state: VState,
        P: MTangent,
        meas: sf.V3,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V3, sf.V3, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    innov = meas_pred - meas

    innov_var = sf.V3()
    Hx = jacobian_chain_rule(meas_pred[0], state)
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    Hy = jacobian_chain_rule(meas_pred[1], state)
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]
    Hz = jacobian_chain_rule(meas_pred[2], state)
    innov_var[2] = (Hz * P * Hz.T + R)[0,0]

    return (innov, innov_var, Hx.T)

def compute_mag_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    H = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_z_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    H = jacobian_chain_rule(meas_pred[2], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    q = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
    r = sf.Quaternion.symbolic('r')
    delta_q = q * r.conj() # create a quaternion error of the measurement at the origin
    delta_meas_pred = 2 * delta_q.z # Use small angle approximation to obtain a simpler jacobian

    H = jacobian_chain_rule(delta_meas_pred, state)
    H = H.subs({r.w: q.w, r.x: q.x, r.y: q.y, r.z: q.z}) # assume innovation is small

    for i in range(State.tangent_dim()):
        H[i] = sp.factor(H[i]).subs(q.w**2 + q.x**2 + q.y**2 + q.z**2, 1) # unit norm quaternion
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_declination_pred_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = sf.atan2(state["mag_I"][1], state["mag_I"][0], epsilon=epsilon)

    H = jacobian_chain_rule(meas_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def compute_gnss_yaw_pred_innov_var_and_h(
        state: VState,
        P: MTangent,
        antenna_yaw_offset: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VTangent):

    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"]

    # define antenna vector in body frame
    ant_vec_bf = sf.V3(sf.cos(antenna_yaw_offset), sf.sin(antenna_yaw_offset), 0)

    # rotate into earth frame
    ant_vec_ef = R_to_earth * ant_vec_bf

    # Calculate the yaw angle from the projection
    meas_pred = sf.atan2(ant_vec_ef[1], ant_vec_ef[0], epsilon=epsilon)

    H = jacobian_chain_rule(meas_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def predict_gravity_direction(state: State):
    # get transform from earth to body frame
    R_to_body = state["quat_nominal"].inverse()

    # the innovation is the error between measured acceleration
    #  and predicted (body frame), assuming no body acceleration
    return R_to_body * sf.Matrix([0,0,-1])

def compute_gravity_xyz_innov_var_and_hx(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # initialize outputs
    innov_var = sf.V3()
    H = [None] * 3

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    #  for each axis
    for i in range(3):
        H[i] = jacobian_chain_rule(meas_pred[i], state)
        innov_var[i] = (H[i] * P * H[i].T + R)[0,0]

    return (innov_var, H[0].T)

def compute_gravity_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent, VTangent, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    H = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_gravity_z_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent, VTangent, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    H = jacobian_chain_rule(meas_pred[2], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

# GSF yaw 原先由独立 derivation_yaw_estimator.py 生成。Dima N1 将它并入
# 本脚本，使主 EKF 与 GSF 共享一个符号生成入口和一个 generated 目录。
YawEstimatorState = Values(
    vel = sf.V2(),
    R = sf.Rot2()
)

class VYawEstimatorTangent(sf.Matrix):
    SHAPE = (YawEstimatorState.tangent_dim(), 1)

class MYawEstimatorTangent(sf.Matrix):
    SHAPE = (YawEstimatorState.tangent_dim(), YawEstimatorState.tangent_dim())

def yaw_est_rot2_small_angle(angle: sf.V1):
    # 小角度复数旋转避免预测中引入三角函数，同时保持 PX4 GSF 的角度回绕语义。
    return sf.Rot2(sf.Complex(1, angle[0]))

def yaw_est_predict_covariance(
        state: VYawEstimatorTangent,
        P: MYawEstimatorTangent,
        d_vel: sf.V2,
        d_vel_var: sf.Scalar,
        d_ang: sf.Scalar,
        d_ang_var: sf.Scalar,
):
    state = YawEstimatorState.from_tangent(state)
    d_ang = sf.V1(d_ang)

    state_error = Values(
        vel = sf.V2.symbolic("delta_vel"),
        yaw = sf.V1.symbolic("delta_yaw")
    )
    state_t = Values(
        vel = state["vel"] + state_error["vel"],
        R = state["R"] * yaw_est_rot2_small_angle(state_error["yaw"])
    )
    noise = Values(
        d_vel = sf.V2.symbolic("a_n"),
        d_ang = sf.V1.symbolic("w_n"),
    )
    input_t = Values(
        d_vel = d_vel - noise["d_vel"],
        d_ang = d_ang - noise["d_ang"]
    )
    state_t_pred = Values(
        vel = state_t["vel"] + state_t["R"] * input_t["d_vel"],
        R = state_t["R"] * yaw_est_rot2_small_angle(input_t["d_ang"])
    )
    state_pred = Values(
        vel = state["vel"] + state["R"] * d_vel,
        R = state["R"] * yaw_est_rot2_small_angle(d_ang)
    )
    delta_rot = state_pred["R"].inverse() * state_t_pred["R"]
    state_error_pred = Values(
        vel = state_t_pred["vel"] - state_pred["vel"],
        yaw = sf.simplify(delta_rot.z.imag)
    )

    zero_state_error = {
        state_error[key]: state_error[key].zero()
        for key in state_error.keys()
    }
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}
    F = VYawEstimatorTangent(state_error_pred.to_storage()).jacobian(
        state_error).subs(zero_state_error).subs(zero_noise)
    G = VYawEstimatorTangent(state_error_pred.to_storage()).jacobian(
        noise).subs(zero_state_error).subs(zero_noise)
    var_u = sf.Matrix.diag([d_vel_var, d_vel_var, d_ang_var])
    P_new = F * P * F.T + G * var_u * G.T

    for index in range(YawEstimatorState.tangent_dim()):
        for j in range(YawEstimatorState.tangent_dim()):
            if index > j:
                P_new[index,j] = 0
    return P_new

def yaw_est_compute_measurement_update(
        P: MYawEstimatorTangent,
        vel_obs_var: sf.Scalar,
        epsilon: sf.Scalar
):
    H = sf.Matrix([[1, 0, 0], [0, 1, 0]])
    R = sf.Matrix([[vel_obs_var, 0], [0, vel_obs_var]])
    S = H * P * H.T + R
    S_det = S[0, 0] * S[1, 1] - S[1, 0] * S[0, 1]
    S_det_inv = add_epsilon_sign(1 / S_det, S_det, epsilon)
    S_inv = sf.M22([[S[1, 1], -S[0, 1]],
                    [-S[1, 0], S[0, 0]]]) * S_det_inv
    K = (P * H.T) * S_inv
    P_new = P - K * H * P

    for index in range(YawEstimatorState.tangent_dim()):
        for j in range(YawEstimatorState.tangent_dim()):
            if index > j:
                P_new[index,j] = 0
    return (S_inv, S_det_inv, K, P_new)

print("Derive EKF2 equations...")
generate_px4_function(predict_covariance, output_names=None)
generate_px4_function(compute_mag_declination_pred_innov_var_and_h, output_names=["pred", "innov_var", "H"])
generate_px4_function(compute_mag_innov_innov_var_and_hx, output_names=["innov", "innov_var", "Hx"])
generate_px4_function(compute_mag_y_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_mag_z_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_gnss_yaw_pred_innov_var_and_h, output_names=["meas_pred", "innov_var", "H"])
generate_px4_function(compute_gravity_xyz_innov_var_and_hx, output_names=["innov_var", "Hx"])
generate_px4_function(compute_gravity_y_innov_var_and_h, output_names=["innov_var", "Hy"])
generate_px4_function(compute_gravity_z_innov_var_and_h, output_names=["innov_var", "Hz"])
generate_px4_function(yaw_est_predict_covariance, output_names=None)
generate_px4_function(yaw_est_compute_measurement_update,
                      output_names=["S_inv", "S_det_inv", "K", "P_new"])
generate_px4_state(State, tangent_idx)
